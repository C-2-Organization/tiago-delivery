import re
from typing import Optional

# 기본: 3자리 호수 "101호" / "101 호" / "101"
ROOM_RE = re.compile(r'(\d{3})\s*호?')

def parse_destination_id(text: str) -> Optional[str]:
    if not text:
        return None
    m = ROOM_RE.search(text)
    if not m:
        return None
    return m.group(1)
