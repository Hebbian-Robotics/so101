# Verified upstream license copies

These runtime npm distributions omit their license files. Copies are preserved from pinned upstream sources:

- [`@react-three/fiber` 9.8.2](https://github.com/pmndrs/react-three-fiber/blob/v9.8.2/LICENSE).
- [`react-remove-scroll-bar` 2.3.8](https://www.npmjs.com/package/react-remove-scroll-bar/v/2.3.8) declares MIT. The [upstream MIT license](https://github.com/theKashey/react-remove-scroll-bar/blob/8ca9ba5ea52de03308fe8ced94f7b159a44d28ff/LICENSE) is pinned to the source repository because the npm release has no corresponding public tag.
- [`urdf-loader` 0.13.1](https://github.com/gkjohnson/urdf-loaders/tree/v0.13.1).

The build includes these texts and installed dependencies’ license and notice files in `THIRD_PARTY_NOTICES.txt`. New versions require verifying the corresponding upstream licenses again.

Components copied from the shadcn/ui registry carry the [upstream MIT license](https://github.com/shadcn-ui/ui/blob/f50887ae68d0fdcbfbe86b57be08af94a4c88a68/LICENSE.md). The generated notices also include the licenses for the CSS shipped from Tailwind and tw-animate-css.
