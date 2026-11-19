import { execFileSync } from "node:child_process";
import { existsSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { z } from "zod";

const licenseRecord = z.object({
  name: z.string(),
  versions: z.array(z.string()),
  paths: z.array(z.string()),
  license: z.string(),
});
const records = z
  .record(z.string(), z.array(licenseRecord))
  .parse(
    JSON.parse(
      execFileSync(
        "pnpm",
        [
          "--filter",
          "@hebbian/so101-calibrator-ui",
          "licenses",
          "list",
          "--prod",
          "--json",
        ],
        { encoding: "utf8" },
      ),
    ),
  );
const notices = [
  "Runtime dependency licenses for the SO-101 setup browser. Robot model licensing is in robot/so101/LICENSE.txt.",
  "\nshadcn/ui registry components (MIT)\n",
  readFileSync(
    resolve(import.meta.dirname, "../licenses/shadcn-ui/LICENSE"),
    "utf8",
  ),
];
for (const cssDependency of ["tailwindcss", "tw-animate-css"]) {
  notices.push(
    `\n${cssDependency} stylesheet (MIT)\n`,
    readFileSync(
      resolve(import.meta.dirname, "../node_modules", cssDependency, "LICENSE"),
      "utf8",
    ),
  );
}
for (const record of Object.values(records).flat()) {
  notices.push(
    `\n${record.name} ${record.versions.join(", ")} (${record.license})\n`,
  );
  let licenseCount = 0;
  const seenTexts = new Set<string>();
  const licenseDirectories = [...record.paths];
  for (const version of record.versions) {
    const packageKey = `${record.name.replace(/^@/, "").replaceAll("/", "-")}-${version}`;
    const fallbackDirectory = resolve(
      import.meta.dirname,
      "../licenses",
      packageKey,
    );
    if (existsSync(fallbackDirectory))
      licenseDirectories.push(fallbackDirectory);
  }
  for (const directory of licenseDirectories) {
    for (const entry of readdirSync(directory, { withFileTypes: true })) {
      if (
        !entry.isFile() ||
        !/^(?:licen[sc]e|copying|notice)/i.test(entry.name)
      )
        continue;
      const content = readFileSync(resolve(directory, entry.name), "utf8");
      if (!seenTexts.has(content)) {
        notices.push(content);
        seenTexts.add(content);
      }
      licenseCount += 1;
    }
  }
  if (!licenseCount)
    throw new Error(
      `No license text found for runtime dependency ${record.name}`,
    );
}
writeFileSync(
  resolve(
    import.meta.dirname,
    "../../src/so101_setup/assets/web/THIRD_PARTY_NOTICES.txt",
  ),
  notices.join("\n"),
);
