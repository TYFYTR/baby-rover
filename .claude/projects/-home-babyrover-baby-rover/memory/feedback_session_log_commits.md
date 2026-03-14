---
name: Session log entry before commits
description: Before committing, write a session log entry in docs/session_log.md and append the session log ID (date-time) to the commit message with a pipe separator
type: feedback
---

Always write a session log entry in `docs/session_log.md` before committing. The session log ID format is `YYYY-MM-DD-HH-MM`. Append it to the end of the commit subject line as `| Session Log YYYY-MM-DD-HH-MM`.

**Why:** Alex uses the session log as a project diary and wants commits and log entries cross-referenced.

**How to apply:** Any time the user asks to commit, first add a session log entry covering what was done, decisions made, and what's next. Then reference that session ID in the commit message.
