import React, { useState } from 'react';
import OriginalDocItem from '@theme-original/DocItem';
import type { Props } from '@theme-original/DocItem';

import TranslateButton from '@site/src/components/TranslateButton/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';

export default function DocItemWrapper(props: Props): JSX.Element {
  const { content } = props;
  const metadata = content?.metadata ?? {};

  const [modifiedHTML, setModifiedHTML] = useState<string | null>(null);
  const [isRTL, setIsRTL] = useState(false);

  // This is the CORRECT way to get the original Markdown source
  const originalMarkdown = content?.raw ?? '';

  // Bug 1 Fix: Accept explicit isUrdu flag - only TranslateButton should pass true
  // PersonalizeButton should always pass false since personalization is never Urdu
  const handleContentChange = (newMarkdown: string, isUrdu: boolean = false) => {
    // Only set RTL if explicitly marked as Urdu (from TranslateButton)
    setIsRTL(isUrdu);
    if (newMarkdown && (isUrdu || newMarkdown !== originalMarkdown)) {
      setModifiedHTML(convertMarkdownToHTML(newMarkdown));
    } else {
      setModifiedHTML(null); // Go back to original
    }
  };

  // Hide buttons on homepage, blog, or invalid pages
  const shouldHideUI =
    !metadata.permalink ||
    metadata.permalink === '/' ||
    (metadata.id && metadata.id.includes('blog'));

  if (shouldHideUI) {
    return <OriginalDocItem {...props} />;
  }

  return (
    <>
      {/* Control Panel */}
      <div
        style={{
          marginBottom: '2rem',
          padding: '1rem',
          background: 'var(--ifm-card-background-color)',
          borderRadius: '12px',
          border: '1px solid var(--ifm-color-emphasis-300)',
          boxShadow: '0 4px 12px rgba(0,0,0,0.05)',
          display: 'flex',
          gap: '1rem',
          flexWrap: 'wrap',
          alignItems: 'center',
        }}
      >
        <PersonalizeButton
          originalContent={originalMarkdown}
          chapterId={metadata.id ?? ''}
          onContentChange={handleContentChange}
        />
        <TranslateButton
          originalContent={originalMarkdown}
          chapterId={metadata.id ?? ''}
          onContentChange={handleContentChange}
        />
      </div>

      {/* Content with RTL support */}
      <div dir={isRTL ? 'rtl' : 'ltr'} className={isRTL ? 'rtl-text' : ''}>
        {modifiedHTML ? (
          <article
            className="markdown"
            dangerouslySetInnerHTML={{ __html: modifiedHTML }}
          />
        ) : (
          <OriginalDocItem {...props} />
        )}
      </div>
    </>
  );
}

// HTML escape function to prevent XSS attacks (Bug 2 Fix)
function escapeHTML(text: string): string {
  const map: Record<string, string> = {
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#x27;',
    '/': '&#x2F;',
  };
  return text.replace(/[&<>"'/]/g, (char) => map[char]);
}

// Fast & reliable Markdown → HTML converter (perfect for Urdu/personalized content)
function convertMarkdownToHTML(markdown: string): string {
  if (!markdown) return '';

  // Split into lines for better processing
  const lines = markdown.split('\n');
  const htmlParts: string[] = [];
  let inCodeBlock = false;
  let codeBlockContent: string[] = [];
  let currentParagraph: string[] = [];
  let currentListItems: string[] = [];

  const flushParagraph = () => {
    if (currentParagraph.length > 0) {
      htmlParts.push(`<p>${processInlineMarkdown(currentParagraph.join(' '))}</p>`);
      currentParagraph = [];
    }
  };

  const flushList = () => {
    if (currentListItems.length > 0) {
      htmlParts.push(`<ul>${currentListItems.map(item => `<li>${processInlineMarkdown(item)}</li>`).join('')}</ul>`);
      currentListItems = [];
    }
  };

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];

    // Handle code blocks
    if (line.trim().startsWith('```')) {
      if (inCodeBlock) {
        // End code block
        htmlParts.push(`<pre><code>${escapeHTML(codeBlockContent.join('\n'))}</code></pre>`);
        codeBlockContent = [];
        inCodeBlock = false;
      } else {
        // Start code block - flush any pending content
        flushParagraph();
        flushList();
        inCodeBlock = true;
      }
      continue;
    }

    if (inCodeBlock) {
      codeBlockContent.push(line);
      continue;
    }

    // Handle headings (Bug 3 Fix: headings should not be wrapped in <p> tags)
    const h3Match = line.match(/^###\s+(.+)$/);
    if (h3Match) {
      flushParagraph();
      flushList();
      htmlParts.push(`<h3>${processInlineMarkdown(h3Match[1])}</h3>`);
      continue;
    }
    const h2Match = line.match(/^##\s+(.+)$/);
    if (h2Match) {
      flushParagraph();
      flushList();
      htmlParts.push(`<h2>${processInlineMarkdown(h2Match[1])}</h2>`);
      continue;
    }
    const h1Match = line.match(/^#\s+(.+)$/);
    if (h1Match) {
      flushParagraph();
      flushList();
      htmlParts.push(`<h1>${processInlineMarkdown(h1Match[1])}</h1>`);
      continue;
    }

    // Handle lists (Bug 3 Fix: collect consecutive list items into single <ul>)
    const listMatch = line.match(/^-\s+(.+)$/);
    if (listMatch) {
      flushParagraph();
      currentListItems.push(listMatch[1]);
      continue;
    }

    // Handle empty lines (paragraph/list breaks)
    if (line.trim() === '') {
      flushParagraph();
      flushList();
      continue;
    }

    // Regular text line - add to current paragraph
    flushList(); // Lists and paragraphs don't mix
    currentParagraph.push(line);
  }

  // Flush remaining content
  flushParagraph();
  flushList();

  return htmlParts.join('\n');
}

// Process inline markdown (bold, italic, links) with HTML escaping
function processInlineMarkdown(text: string): string {
  // Escape HTML first to prevent XSS (Bug 2 Fix)
  let html = escapeHTML(text);

  // Process markdown inline elements (order matters - process bold before italic)
  // Bold: **text** or __text__
  html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
  html = html.replace(/__(.*?)__/g, '<strong>$1</strong>');

  // Italic: *text* or _text_ (process after bold to avoid conflicts)
  // Since bold (**text** and __text__) is already processed,
  // remaining single asterisks/underscores can be treated as italic
  // Simple pattern: match *text* or _text_ where text doesn't contain * or _
  html = html.replace(/\*([^*\n]+?)\*/g, '<em>$1</em>');
  html = html.replace(/_([^_\n]+?)_/g, '<em>$1</em>');

  // Links: [text](url) - escape URL to prevent javascript: attacks (Bug 2 Fix)
  html = html.replace(/\[(.*?)\]\((.*?)\)/g, (match, linkText, url) => {
    // Escape URL and validate it's not javascript: or data: protocol
    const escapedUrl = escapeHTML(url);
    // Additional safety: if it starts with javascript:, data:, or vbscript:, convert to #
    const safeUrl = /^(javascript|data|vbscript):/i.test(escapedUrl) ? '#' : escapedUrl;
    return `<a href="${safeUrl}">${linkText}</a>`;
  });

  return html;
}