/**
 * Get the base path for the site
 * This ensures consistent path handling across the site
 */
export function getBasePath(): string {
  return "/2025-Team-The_Winners-Docs";
}

/**
 * Prepend the base path to a URL if it doesn't already start with it
 * @param url - The URL to process
 * @returns The URL with base path prepended
 */
export function withBasePath(url: string): string {
  const base = getBasePath();
  
  // If URL is empty or already has base path, return as-is
  if (!url || url.startsWith(base)) {
    return url;
  }
  
  // If URL starts with http/https, return as-is (external URL)
  if (url.startsWith('http://') || url.startsWith('https://')) {
    return url;
  }
  
  // Ensure URL starts with / and prepend base path
  const normalizedUrl = url.startsWith('/') ? url : `/${url}`;
  return `${base}${normalizedUrl}`;
}

/**
 * Get a full image URL with base path
 * @param imagePath - The image path (e.g., "images/img1.jpg")
 * @returns The full image URL with base path
 */
export function getImageUrl(imagePath: string): string {
  return withBasePath(`/${imagePath}`);
}

