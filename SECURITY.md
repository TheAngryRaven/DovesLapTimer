# Security Policy

DovesLapTimer is an embedded lap-timing library — it parses no untrusted
network input and runs on a hobbyist's microcontroller, so the attack surface
is small. Still, if you find a security-relevant issue (e.g. a buffer overflow
reachable from crafted GPS input, or a problem in an example sketch that could
affect a connected device), we'd like to know.

## Supported versions

Only the latest released minor version receives fixes. Given the library's
scope, older versions are not back-patched — update to the current release.

| Version | Supported |
| ------- | --------- |
| 4.1.x   | ✅        |
| < 4.1   | ❌        |

## Reporting a vulnerability

Please **do not** open a public issue for a security problem.

Use GitHub's private vulnerability reporting:
**Security → Report a vulnerability** on the repository, or contact the
maintainer directly. Include:

- The version (or commit) affected
- A description of the issue and its impact
- Steps to reproduce, ideally with a minimal sketch or GPS data sample

We'll acknowledge within a reasonable time and work with you on a fix and
coordinated disclosure. Since this is a volunteer-maintained hobby project,
please be patient with response times.
