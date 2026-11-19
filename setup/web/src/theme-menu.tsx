import { ChevronDown, Monitor, Moon, Sun } from "lucide-react";
import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuRadioGroup,
  DropdownMenuRadioItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";

const themeOptions = {
  system: { label: "System", Icon: Monitor },
  light: { label: "Light", Icon: Sun },
  dark: { label: "Dark", Icon: Moon },
};

export type ThemePreference = keyof typeof themeOptions;

export function ThemeMenu({
  theme,
  onThemeChange,
}: {
  theme: ThemePreference;
  onThemeChange: (theme: ThemePreference) => void;
}) {
  const { Icon } = themeOptions[theme];
  return (
    <TooltipProvider delayDuration={400}>
      <DropdownMenu modal={false}>
        <Tooltip>
          <TooltipTrigger asChild>
            <DropdownMenuTrigger asChild>
              <Button
                className="theme-trigger h-11 min-w-11 gap-1 px-2 text-muted-foreground"
                type="button"
                variant="ghost"
                aria-label={`Theme: ${themeOptions[theme].label}`}
              >
                <Icon aria-hidden="true" />
                <ChevronDown className="size-3" aria-hidden="true" />
              </Button>
            </DropdownMenuTrigger>
          </TooltipTrigger>
          <TooltipContent sideOffset={8} collisionPadding={12}>
            Change theme
          </TooltipContent>
        </Tooltip>
        <DropdownMenuContent
          className="min-w-42 rounded-[10px]"
          align="end"
          sideOffset={8}
          collisionPadding={12}
        >
          <DropdownMenuRadioGroup value={theme} aria-label="Color theme">
            {(["system", "light", "dark"] as const).map((preference) => {
              const { Icon, label } = themeOptions[preference];
              return (
                <DropdownMenuRadioItem
                  className="min-h-10"
                  key={preference}
                  value={preference}
                  onSelect={() => onThemeChange(preference)}
                >
                  <Icon aria-hidden="true" />
                  {label}
                </DropdownMenuRadioItem>
              );
            })}
          </DropdownMenuRadioGroup>
        </DropdownMenuContent>
      </DropdownMenu>
    </TooltipProvider>
  );
}
