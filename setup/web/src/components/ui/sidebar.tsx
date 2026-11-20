// Static sidebar primitives adapted from shadcn/ui's Base Vega registry.
import { useRender } from "@base-ui/react/use-render";
import type * as React from "react";
import { cn } from "@/lib/utils";

function Sidebar({ className, ...props }: React.ComponentProps<"header">) {
  return (
    <header
      data-slot="sidebar"
      className={cn(
        "flex flex-col bg-sidebar text-sidebar-foreground",
        className,
      )}
      {...props}
    />
  );
}

function SidebarHeader({ className, ...props }: React.ComponentProps<"div">) {
  return (
    <div
      data-slot="sidebar-header"
      data-sidebar="header"
      className={cn("flex flex-col gap-2 p-2", className)}
      {...props}
    />
  );
}

function SidebarFooter({ className, ...props }: React.ComponentProps<"div">) {
  return (
    <div
      data-slot="sidebar-footer"
      data-sidebar="footer"
      className={cn("flex flex-col gap-2 p-2", className)}
      {...props}
    />
  );
}

function SidebarMenu({ className, ...props }: React.ComponentProps<"ul">) {
  return (
    <ul
      data-slot="sidebar-menu"
      data-sidebar="menu"
      className={cn("flex w-full min-w-0 flex-col gap-1", className)}
      {...props}
    />
  );
}

function SidebarMenuItem({ className, ...props }: React.ComponentProps<"li">) {
  return (
    <li
      data-slot="sidebar-menu-item"
      data-sidebar="menu-item"
      className={cn("group/menu-item relative", className)}
      {...props}
    />
  );
}

function SidebarMenuButton({
  render,
  isActive = false,
  className,
  ...props
}: useRender.ComponentProps<"button"> & {
  isActive?: boolean;
}) {
  return useRender({
    defaultTagName: "button",
    render,
    props: {
      ...props,
      className: cn(
        "peer/menu-button flex w-full items-center gap-2 rounded-md p-2 text-left text-sm ring-sidebar-ring outline-hidden hover:bg-sidebar-accent hover:text-sidebar-accent-foreground focus-visible:ring-2 data-active:bg-sidebar-accent data-active:font-medium data-active:text-sidebar-accent-foreground [&>svg]:size-4 [&>svg]:shrink-0",
        className,
      ),
    },
    state: {
      slot: "sidebar-menu-button",
      sidebar: "menu-button",
      active: isActive,
    },
  });
}

export {
  Sidebar,
  SidebarFooter,
  SidebarHeader,
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
};
