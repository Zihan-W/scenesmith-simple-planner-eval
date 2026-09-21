"""Compatibility imports for the former PickLift-only generation entry point.

All generation behavior lives in :mod:`bt_generation`.
"""

from examples.online_manipulation.bt_generation import (  # noqa: F401
    CAMERA_ORDER,
    LEGACY_REQUEST_SCHEMA as REQUEST_SCHEMA,
    LEGACY_RESULT_SCHEMA as RESULT_SCHEMA,
    ChatCompletion,
    GenerationError,
    LoadedRequest,
    OpenAICompatibleChatClient,
    build_messages,
    generate,
    load_request,
    main,
    write_result,
)

if __name__ == "__main__":
    main()
