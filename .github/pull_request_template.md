<!--
Thanks for contributing! Keep PRs focused on one logical change.
See CONTRIBUTING.md for the full workflow and testing philosophy.
-->

## Summary

<!-- What does this change and why? -->

## Type of change

- [ ] Bug fix
- [ ] New feature
- [ ] Refactor (no behavior change)
- [ ] Docs / examples
- [ ] CI / tooling

## Testing

<!-- How did you verify this? -->

- [ ] `cd test && make run` passes locally
- [ ] Added/updated tests in the appropriate layer (unit / NMEA replay), or N/A
- [ ] If timing output changed intentionally, updated the pinned goldens **and** explained why below

## Checklist

- [ ] Doc comments updated for any public API changes (feeds the API docs site)
- [ ] `README.md` updated if user-facing behavior changed, or N/A
- [ ] `CHANGELOG.md` updated under "Unreleased", or N/A
- [ ] No new heap allocation in the GPS hot path; SRAM-conscious for AVR targets

## Notes for reviewers

<!-- Anything to call out: tradeoffs, follow-ups, areas you're unsure about. -->
