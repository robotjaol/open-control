# Security policy

This repository contains educational simulations and documentation. It is not safety-rated software and
must not be used to protect people, machines or processes. See [hardware lab](docs/hardware-lab.md).

## Reporting a vulnerability
Please use GitHub's private vulnerability reporting for this repository (Security tab, "Report a
vulnerability"). If that is unavailable, open an issue that states only that you have a security
concern, without details. Reports are handled on a best-effort basis; there is no service-level commitment.

## Scope
- Code in `src/`, `tools/`, `scripts/` and the preview site in `app/`.
- The preview site renders text from JSON and markdown after HTML-escaping and has no server component.
- Dependencies are audited by the `dependency-audit` workflow (not yet run).

## Industrial control cybersecurity
Cybersecurity of industrial control systems is a planned topic with no content here. Nothing in this
repository is guidance for securing a plant network.
