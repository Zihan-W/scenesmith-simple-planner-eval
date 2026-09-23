"""Complete, ordered TAMP model transcripts with strict offline replay.

Matching includes model options, prompts and image bytes. File locations are not
model inputs. A miss is an experiment divergence, never a live-provider fallback.
"""
import dataclasses
import hashlib
import json
import mimetypes
from pathlib import Path
import time

from planner.src.bt.generation import ChatCompletion, GenerationError, ProviderError


class ReplayDivergence(RuntimeError):
    """The actual model request sequence differs from the recorded experiment."""


def canonical_request(*, model, messages, temperature=None, response_format=None):
    """Return the actual semantic request, verifying and hashing image content."""
    messages = json.loads(json.dumps(messages))
    for message in messages:
        if not isinstance(message.get('content'), list):
            continue
        for item in message['content']:
            if item.get('type') != 'image_url':
                continue
            image = item['image_url']
            if isinstance(image, dict) and 'path' in image:
                path = Path(image['path'])
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                if digest != image['sha256']:
                    raise ReplayDivergence('Observation image changed before model request')
                item['image_url'] = {'sha256': digest,
                    'mime_type': image.get('mime_type') or mimetypes.guess_type(path.name)[0]}
    return {'model': model, 'messages': messages, 'temperature': temperature,
            'response_format': response_format}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':'), allow_nan=False).encode()).hexdigest()


class TranscriptChatClient:
    """Record every response/error, or replay an exact sequence without a network client."""

    def __init__(self, *, output, client=None, replay=None):
        if (client is None) == (replay is None):
            raise ValueError('Provide exactly one live client or replay transcript')
        self.client = client
        self.output = Path(output)
        self.replay = Path(replay) if replay is not None else None
        self.index = 0
        self.live_calls = 0
        self.deadline_monotonic_s = None
        self.timeout_s = getattr(client, 'timeout_s', 180.)
        self.failure = None
        self.entries = []
        self.replay_sha256 = None
        if self.replay is not None:
            raw = self.replay.read_bytes()
            self.replay_sha256 = hashlib.sha256(raw).hexdigest()
            rows = [json.loads(line) for line in raw.splitlines()]
            if not rows or rows[0].get('schema') != 'tamp.model_transcript.v1':
                raise ValueError('Unsupported model transcript schema')
            header, self.entries = rows[0], rows[1:]
            for index, entry in enumerate(self.entries):
                if entry.get('index') != index or entry.get('request_sha256') != digest(entry['request']):
                    raise ValueError('Corrupt model transcript request')
                if ('response' in entry) == ('error' in entry):
                    raise ValueError('Incomplete model transcript outcome')
                if entry.get('entry_sha256') != digest({k:v for k,v in entry.items() if k != 'entry_sha256'}):
                    raise ValueError('Corrupt model transcript entry')
        else:
            header = {'schema': 'tamp.model_transcript.v1', 'transport': {
                'max_tokens': client.max_tokens, 'token_parameter': client.token_parameter,
                'base_url': client.base_url}}
        # No credentials or authorization headers are recorded.
        with self.output.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(header, ensure_ascii=False) + '\n')

    def set_deadline(self, deadline_monotonic_s):
        """Participate in the same run/stage budgets as the live transport."""
        self.deadline_monotonic_s = deadline_monotonic_s

    def _append(self, entry):
        entry['entry_sha256'] = digest(entry)
        with self.output.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
            stream.flush()

    def _diverge(self, reason):
        self.failure = reason
        raise ReplayDivergence(reason)

    def complete(self, *, model, messages, temperature=None, response_format=None):
        """Match each invocation, including invalid responses and provider retries."""
        if self.deadline_monotonic_s is not None and time.perf_counter() >= self.deadline_monotonic_s:
            raise GenerationError('Model request wall-time budget exhausted')
        kwargs = dict(model=model, messages=messages, temperature=temperature,
                      response_format=response_format)
        try:
            request = canonical_request(**kwargs)
        except ReplayDivergence as error:
            self._diverge(str(error))
        request_hash = digest(request)
        if self.replay is not None:
            if self.index >= len(self.entries):
                self._diverge(f'Model replay exhausted at request {self.index}')
            entry = dict(self.entries[self.index])
            if entry['request_sha256'] != request_hash:
                self._diverge(f'Model replay request {self.index} differs: '
                              f"expected {entry['request_sha256']}, actual {request_hash}")
            self.index += 1
            entry.pop('entry_sha256')
            self._append(entry)
            if 'error' in entry:
                error = entry['error']
                if error['type'] == 'ProviderError':
                    raise ProviderError(error['message'], kind=error['kind'],
                                        retryable=error['retryable'], status_code=error['status_code'])
                raise GenerationError(error['message'])
            return ChatCompletion(**entry['response'])
        self.client.timeout_s = self.timeout_s
        self.client.set_deadline(self.deadline_monotonic_s)
        entry = {'index': self.index, 'request': request, 'request_sha256': request_hash}
        start = time.perf_counter()
        self.live_calls += 1
        try:
            response = self.client.complete(**kwargs)
        except GenerationError as error:
            entry['error'] = {'type': type(error).__name__, 'message': str(error)}
            if isinstance(error, ProviderError):
                entry['error'].update(error.as_dict())
            raise
        else:
            entry['response'] = dataclasses.asdict(response)
            return response
        finally:
            entry['provider_duration_s'] = time.perf_counter() - start
            self._append(entry)
            self.index += 1

    def assert_consumed(self):
        """Reject early termination as a complete replay of a longer transcript."""
        if self.replay is not None and self.index != len(self.entries):
            self._diverge(f'Model replay consumed {self.index} of {len(self.entries)} requests')

    def evidence(self):
        """Separate logical model invocations from actual provider calls."""
        return {'mode': 'strict_replay' if self.replay else 'live_record',
                'logical_calls': self.index, 'live_provider_calls': self.live_calls,
                'replay_sha256': self.replay_sha256,
                'transcript_sha256': hashlib.sha256(self.output.read_bytes()).hexdigest(),
                'recorded_calls': len(self.entries) if self.replay else self.index,
                'unused_recorded_calls': len(self.entries) - self.index if self.replay else 0,
                'replay_scope': ('complete_recorded_sequence' if self.index == len(self.entries)
                                 else 'matched_request_prefix') if self.replay else None,
                'divergence': self.failure,
                'timing': 'recorded_provider_latency_not_replayed'}
