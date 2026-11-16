# Contributing

## Development

Install Rust 1.92 or newer, Python 3.12 or newer, Node 24 and pnpm 12.
Linux builds use mold (`sudo apt-get install mold`); macOS and Windows use their
default linker. Then run `pnpm install` and `uv sync --project setup`.
Run `cargo test --manifest-path runtime/Cargo.toml` and
`uv run --project setup pytest` after relevant changes. Build the UI with
`pnpm build` before `uv build --project setup`; run the installed-wheel check
with `uv run --project setup python setup/tools/check-wheel.py dist`.

## Release the Python setup package

Update `setup/pyproject.toml` and `setup/uv.lock`, then push a matching version
tag, for example `git tag v0.1.1` and `git push origin v0.1.1`. The release
workflow builds and verifies the wheel and source archive before attaching
them to a GitHub Release. This does not publish to PyPI. A manual workflow run
on a branch validates and retains build artifacts without publishing a release.
Reruns preserve existing release assets, since applications pin their checksums.

## Code Quality

```bash
lychee --offline README.md CONTRIBUTING.md docs/*.md runtime/README.md setup/README.md
```

### TypeScript

```bash
pnpm lint       # Biome linting
pnpm typecheck  # TypeScript type checking
pnpm check      # Run all checks
```

### Rust

```bash
cargo clippy --manifest-path runtime/Cargo.toml --all-targets --all-features  # Linting
cargo fmt --manifest-path runtime/Cargo.toml # Formatting
```

### Python

```bash
uv run --project setup ruff check --fix  # Linting with auto-fix
uv run --project setup ruff format       # Code formatting
uv run --project setup ty check          # Type checking
```

## Code Style & Philosophy

### Typing & Pattern Matching

- Prefer **explicit types** over raw dicts -- make invalid states unrepresentable where practical
- Prefer **typed variants over string literals** when the set of valid values is known
- Use **exhaustive pattern matching** (`match` in Python and Rust, `ts-pattern` in TypeScript) so the type checker can verify all cases are handled
- Structure types to enable exhaustive matching when handling variants
- Prefer **shared internal functions over factory patterns** when extracting common logic from hooks or functions -- keep each export explicitly defined for better IDE navigation and readability

### Forward Compatibility

- **Unknown values**: Parse to an explicit `Unknown*` variant (never `None`), log at warn level, preserve raw data, gracefully ignore instead of raising exception

### Self-Documenting Code

- **Verbose naming**: Variable and function naming should read like documentation
- **Strategic comments**: Only for non-obvious logic or architectural decisions; avoid restating what code shows
