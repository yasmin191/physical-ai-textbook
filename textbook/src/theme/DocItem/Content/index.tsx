import React, { type ReactNode } from "react";
import DocItemContent from "@theme-original/DocItem/Content";
import type { Props } from "@theme/DocItem/Content";
import { useDoc } from "@docusaurus/plugin-content-docs/client";
import PodcastPlayer from "@site/src/components/PodcastPlayer";

export default function DocItemContentWrapper(props: Props): ReactNode {
  const { metadata } = useDoc();

  // Extract the chapter slug from the doc id (e.g. "module-1-ros2/01-intro-physical-ai" -> "01-intro-physical-ai")
  const docId = metadata.id || "";
  const chapterSlug = docId.includes("/") ? docId.split("/").pop()! : docId;

  return (
    <>
      <PodcastPlayer chapterSlug={chapterSlug} chapterTitle={metadata.title} />
      <DocItemContent {...props} />
    </>
  );
}
