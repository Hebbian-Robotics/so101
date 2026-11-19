# Contributing

## Development

Install Rust 1.92 or newer, Node 24 and pnpm 12. Linux builds use mold
(`sudo apt-get install mold`). Run `pnpm install --frozen-lockfile` and
`pnpm build` before compiling the setup CLI so Cargo can embed the built UI.

```sh
cargo test --manifest-path runtime/Cargo.toml --locked --all-features
cargo test --manifest-path setup/native/Cargo.toml --locked --all-features
cargo run --manifest-path setup/native/Cargo.toml -- setup
```

The prior Python implementation remains available for hardware comparison.
Its checks use Python 3.12 or newer and `uv sync --project setup`.

## Release the setup CLI

Update the version in `setup/native/Cargo.toml` and its lockfile, then push a
matching version tag. The release workflow builds archives for Linux, macOS and
Windows with the bundled UI. A manual workflow run retains archives without
publishing. Reruns preserve existing release assets.

## Code Quality

```bash
lychee --offline README.md CONTRIBUTING.md runtime/README.md setup/README.md setup/src/so101_setup/COORDINATES.md
```

### TypeScript

```bash
pnpm lint       # Biome linting
pnpm typecheck  # TypeScript type checking
pnpm check      # Run all checks
```

### Rust

```bash
cargo install cargo-machete --version 0.9.2 --locked
cargo install cargo-deny --version 0.20.2 --locked
cargo machete --with-metadata --skip-target-dir runtime
cargo deny --manifest-path runtime/Cargo.toml --config runtime/deny.toml --all-features --exclude-dev --locked check bans
cargo clippy --manifest-path runtime/Cargo.toml --all-targets --all-features -- -D warnings
cargo fmt --manifest-path runtime/Cargo.toml
cargo clippy --manifest-path setup/native/Cargo.toml --all-targets --all-features -- -D warnings
cargo fmt --manifest-path setup/native/Cargo.toml
```

### Python

```bash
uv run --project setup ruff check --fix  # Linting with auto-fix
uv run --project setup ruff format       # Code formatting
uv run --project setup ty check          # Type checking
uv run --project setup lint-imports --config setup/pyproject.toml
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
