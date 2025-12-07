import React, { useState } from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import TranslateButton from '@site/src/components/TranslateButton/TranslateButton';
import { useDoc } from '@docusaurus/theme-common/internal';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  const { metadata } = useDoc();
  const [currentContent, setCurrentContent] = useState<string | null>(null);
  const [isRTL, setIsRTL] = useState(false);

  const handleContentChange = (content: string, isUrdu: boolean) => {
    setCurrentContent(content);
    setIsRTL(isUrdu);

    // Apply RTL class to the article element
    const article = document.querySelector('article');
    if (article) {
      if (isUrdu) {
        article.classList.add('rtl');
      } else {
        article.classList.remove('rtl');
      }
    }

    // Update the article content if translated
    if (isUrdu && content) {
      const contentDiv = document.querySelector('.markdown');
      if (contentDiv) {
        // Parse markdown to HTML (basic implementation)
        // In production, use a proper markdown parser like marked or remark
        contentDiv.innerHTML = convertMarkdownToHTML(content);
      }
    }
  };

  // Get original markdown content from the page
  const getOriginalContent = (): string => {
    const contentDiv = document.querySelector('.markdown');
    if (contentDiv) {
      // This is a simplified approach - in production, you'd want to
      // fetch the original .md file or store it differently
      return contentDiv.textContent || '';
    }
    return '';
  };

  // Only show translate button on actual doc pages (not blog, etc.)
  if (metadata.id === '_blog-post' || metadata.permalink === '/') {
    return <DocItem {...props} />;
  }

  return (
    <>
      <div style={{ marginBottom: '1rem' }}>
        <TranslateButton
          originalContent={getOriginalContent()}
          chapterId={metadata.id}
          onContentChange={handleContentChange}
        />
      </div>
      <DocItem {...props} />
    </>
  );
}

// Basic markdown to HTML converter (for demonstration)
// In production, use a library like 'marked' or 'remark'
function convertMarkdownToHTML(markdown: string): string {
  let html = markdown;

  // Code blocks
  html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre><code>$2</code></pre>');

  // Headings
  html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
  html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

  // Bold
  html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

  // Italic
  html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

  // Links
  html = html.replace(/\[(.*?)\]\((.*?)\)/g, '<a href="$2">$1</a>');

  // Lists
  html = html.replace(/^\- (.*$)/gim, '<li>$1</li>');

  // Paragraphs
  html = html.replace(/\n\n/g, '</p><p>');
  html = '<p>' + html + '</p>';

  return html;
}
