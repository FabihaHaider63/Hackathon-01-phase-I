/**
 * File Management Utilities to Prevent Duplicate Creation
 */

const fs = require('fs').promises;
const path = require('path');
const crypto = require('crypto');

/**
 * Checks if a file already exists
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file exists, false otherwise
 */
async function fileExists(filePath) {
  try {
    await fs.access(filePath);
    return true;
  } catch (error) {
    return false;
  }
}

/**
 * Calculates the similarity between two text contents using a simple algorithm
 * @param {string} content1 - First content to compare
 * @param {string} content2 - Second content to compare
 * @returns {number} Similarity score between 0 and 1 (1 being identical)
 */
function calculateTextSimilarity(content1, content2) {
  // If contents are identical, return 1
  if (content1 === content2) {
    return 1;
  }

  // Normalize the content by removing extra whitespace
  const norm1 = content1.toLowerCase().replace(/\s+/g, ' ').trim();
  const norm2 = content2.toLowerCase().replace(/\s+/g, ' ').trim();

  // If normalized contents are identical, return 1
  if (norm1 === norm2) {
    return 1;
  }

  // Calculate similarity using a simple Jaccard index of words
  const words1 = norm1.split(/\s+/);
  const words2 = norm2.split(/\s+/);

  // Create sets of unique words
  const set1 = new Set(words1);
  const set2 = new Set(words2);

  // Find intersection and union
  const intersection = [...set1].filter(word => set2.has(word));
  const union = new Set([...set1, ...set2]);

  // Calculate Jaccard similarity
  return intersection.length / union.size;
}

/**
 * Checks for similar content in existing files within a directory
 * @param {string} content - Content to check for similarity
 * @param {string} directory - Directory to search in
 * @param {number} threshold - Similarity threshold [0, 1] above which is considered similar
 * @returns {Promise<Array>} Array of similar files with their similarity scores
 */
async function findSimilarFiles(content, directory, threshold = 0.8) {
  const similarFiles = [];

  try {
    const files = await fs.readdir(directory, { withFileTypes: true });
    
    for (const file of files) {
      if (file.isFile() && (file.name.endsWith('.md') || file.name.endsWith('.mdx'))) {
        const filePath = path.join(directory, file.name);
        const existingContent = await fs.readFile(filePath, 'utf-8');
        
        const similarity = calculateTextSimilarity(content, existingContent);
        
        if (similarity >= threshold) {
          similarFiles.push({
            filePath,
            similarity,
            title: extractTitle(existingContent) // Extract title for user-friendly display
          });
        }
      }
    }
  } catch (error) {
    console.error(`Error searching for similar files in ${directory}:`, error.message);
  }

  return similarFiles;
}

/**
 * Extracts the title from markdown content (first # heading or title in frontmatter)
 * @param {string} content - Markdown content to extract title from
 * @returns {string} Extracted title or empty string if not found
 */
function extractTitle(content) {
  // Try to extract title from frontmatter first
  const frontmatterMatch = content.match(/title:\s*["']?([^"'\n\r]+)["']?/);
  if (frontmatterMatch) {
    return frontmatterMatch[1].trim();
  }

  // Try to extract title from first heading
  const headingMatch = content.match(/^#\s+(.+)$/m);
  if (headingMatch) {
    return headingMatch[1].trim();
  }

  return '';
}

/**
 * Checks if content is similar to any existing content in the docs directory
 * @param {string} content - Content to check
 * @param {string} targetDirectory - Directory to check for similar files
 * @param {number} threshold - Similarity threshold (default: 0.8)
 * @returns {Promise<Object>} Result with similarFiles array and recommendation
 */
async function checkForSimilarContent(content, targetDirectory, threshold = 0.8) {
  const similarFiles = await findSimilarFiles(content, targetDirectory, threshold);
  
  return {
    hasSimilarContent: similarFiles.length > 0,
    similarFiles,
    recommendation: similarFiles.length > 0 
      ? 'Consider updating the existing content instead of creating new content'
      : 'Safe to create new content'
  };
}

/**
 * Generates a hash of the content to detect changes
 * @param {string} content - Content to hash
 * @returns {string} SHA-256 hash of the content
 */
function generateContentHash(content) {
  return crypto.createHash('sha256').update(content).digest('hex');
}

/**
 * Reads a file if it exists, otherwise returns null
 * @param {string} filePath - Path to the file to read
 * @returns {Promise<string|null>} File content or null if file doesn't exist
 */
async function readFileIfExists(filePath) {
  if (await fileExists(filePath)) {
    return await fs.readFile(filePath, 'utf-8');
  }
  return null;
}

/**
 * Writes content to a file, but only if it's different from the existing content
 * @param {string} filePath - Path to the file to write
 * @param {string} content - Content to write
 * @returns {Promise<boolean>} True if the file was written (or would have been), false if identical
 */
async function writeContentIfChanged(filePath, content) {
  const existingContent = await readFileIfExists(filePath);
  
  if (existingContent === content) {
    console.log(`Content for ${filePath} is identical, skipping write`);
    return false; // No change needed
  }
  
  // Create directory if it doesn't exist
  const dir = path.dirname(filePath);
  await fs.mkdir(dir, { recursive: true });
  
  await fs.writeFile(filePath, content, 'utf-8');
  console.log(`Content written to ${filePath}`);
  return true;
}

module.exports = {
  fileExists,
  calculateTextSimilarity,
  findSimilarFiles,
  checkForSimilarContent,
  extractTitle,
  generateContentHash,
  readFileIfExists,
  writeContentIfChanged
};