/**
 * Remark plugin to add base path prefix to image URLs in markdown content
 * This fixes 404 errors for images when deployed to GitHub Pages with a base path
 */

const BASE_PATH = '/2025-Team-The_Winners-Docs';

export function remarkAddBasePath() {
  return (tree) => {
    // Visit all image nodes and add base path prefix
    const visit = (node) => {
      if (node.type === 'image') {
        // Only add base path if the URL starts with / and doesn't already have it
        if (node.url && node.url.startsWith('/') && !node.url.startsWith(BASE_PATH)) {
          node.url = BASE_PATH + node.url;
        }
      }
      // Handle links too
      if (node.type === 'link') {
        if (node.url && node.url.startsWith('/') && !node.url.startsWith(BASE_PATH)) {
          // Don't add base path to anchor links (fragment identifiers)
          if (!node.url.startsWith('#')) {
            node.url = BASE_PATH + node.url;
          }
        }
      }
      // Recursively visit children
      if (node.children) {
        node.children.forEach(visit);
      }
    };
    
    visit(tree);
  };
}

