"""Single Ark HTTPS call, run only as a killable subprocess of vlm_action."""

from __future__ import annotations

import json
import os
import sys
from urllib import request


def main() -> int:
    try:
        raw = sys.stdin.buffer.read(24_000_001)
        if len(raw) > 24_000_000:
            return 1
        item = json.loads(raw)
        key = os.environ.get('ABOT_ARK_API_KEY', '')
        model = os.environ.get('ABOT_ARK_MODEL', '')
        if not key or not model:
            return 1
        image_url = 'data:image/jpeg;base64,' + item['jpeg']
        body = json.dumps({
            'model': model,
            'messages': [{'role': 'user', 'content': [
                {'type': 'text', 'text': item['prompt']},
                {'type': 'image_url', 'image_url': {'url': image_url}},
            ]}],
        }).encode('utf-8')
        api_request = request.Request(
            item['endpoint'], data=body,
            headers={'Authorization': 'Bearer ' + key, 'Content-Type': 'application/json'},
            method='POST',
        )
        with request.urlopen(api_request, timeout=max(0.1, float(item['timeout_sec']))) as response:
            result = response.read(1_000_001)
        if len(result) > 1_000_000:
            return 1
        parsed = json.loads(result)
        content = parsed['choices'][0]['message']['content']
        if not isinstance(content, str):
            return 1
        sys.stdout.buffer.write(json.dumps({'text': content.strip()}).encode('utf-8'))
        return 0
    except Exception:
        # No backend exception details are sent to ROS clients or stderr.
        return 1


if __name__ == '__main__':
    sys.exit(main())
