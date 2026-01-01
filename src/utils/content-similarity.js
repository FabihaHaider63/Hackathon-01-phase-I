/**
 * Content Similarity Detection Algorithm
 * Implements advanced similarity detection between content pieces
 */

const natural = require('natural');
const { JSDOM } = require('jsdom');

// Initialize stemmers and tokenizers
const porterStemmer = natural.PorterStemmer;
const { WordTokenizer } = natural;

/**
 * Calculates cosine similarity between two pieces of text
 * @param {string} text1 - First text to compare
 * @param {string} text2 - Second text to compare
 * @returns {number} Cosine similarity score between 0 and 1
 */
function cosineSimilarity(text1, text2) {
  // Preprocess texts
  const tokens1 = preprocessText(text1);
  const tokens2 = preprocessText(text2);

  // Create term frequency vectors
  const tf1 = createTermFrequencyVector(tokens1);
  const tf2 = createTermFrequencyVector(tokens2);

  // Get all unique terms
  const allTerms = new Set([...Object.keys(tf1), ...Object.keys(tf2)]);

  // Create vectors based on terms
  const vector1 = [];
  const vector2 = [];
  for (const term of allTerms) {
    vector1.push(tf1[term] || 0);
    vector2.push(tf2[term] || 0);
  }

  // Calculate cosine similarity
  const dotProduct = vector1.reduce((sum, a, i) => sum + a * vector2[i], 0);
  const magnitude1 = Math.sqrt(vector1.reduce((sum, a) => sum + a * a, 0));
  const magnitude2 = Math.sqrt(vector2.reduce((sum, a) => sum + a * a, 0));

  if (magnitude1 === 0 || magnitude2 === 0) {
    return 0; // If one vector is zero, similarity is 0
  }

  return dotProduct / (magnitude1 * magnitude2);
}

/**
 * Preprocesses text by normalizing and tokenizing
 * @param {string} text - Text to preprocess
 * @returns {Array<string>} Array of processed tokens
 */
function preprocessText(text) {
  // Remove HTML tags if any
  const dom = new JSDOM(text);
  const cleanText = dom.window.document.body.textContent || '';

  // Convert to lowercase and tokenize
  const tokens = new WordTokenizer().tokenize(cleanText.toLowerCase());

  // Remove common stop words and stem the rest
  const stopWords = new Set(['the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'being', 'have', 'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could', 'should']);
  const filteredTokens = tokens
    .filter(token => /^[a-zA-Z]+$/.test(token)) // Only keep alphabetic tokens
    .filter(token => !stopWords.has(token))
    .map(token => porterStemmer.stem(token)); // Stem the tokens

  return filteredTokens;
}

/**
 * Creates a term frequency vector from tokens
 * @param {Array<string>} tokens - Array of tokens
 * @returns {Object} Term frequency vector
 */
function createTermFrequencyVector(tokens) {
  const tf = {};
  for (const token of tokens) {
    tf[token] = (tf[token] || 0) + 1;
  }
  return tf;
}

/**
 * Calculates Jaccard similarity between two pieces of text
 * @param {string} text1 - First text to compare
 * @param {string} text2 - Second text to compare
 * @returns {number} Jaccard similarity score between 0 and 1
 */
function jaccardSimilarity(text1, text2) {
  const tokens1 = new Set(preprocessText(text1));
  const tokens2 = new Set(preprocessText(text2));

  // Calculate intersection and union
  const intersection = [...tokens1].filter(token => tokens2.has(token));
  const union = new Set([...tokens1, ...tokens2]);

  if (union.size === 0) {
    return 0; // If both sets are empty, similarity is 0
  }

  return intersection.length / union.size;
}

/**
 * Calculates a comprehensive similarity score between two pieces of text
 * @param {string} text1 - First text to compare
 * @param {string} text2 - Second text to compare
 * @param {Object} options - Configuration options
 * @param {number} options.cosineWeight - Weight for cosine similarity (default: 0.5)
 * @param {number} options.jaccardWeight - Weight for Jaccard similarity (default: 0.5)
 * @returns {number} Comprehensive similarity score between 0 and 1
 */
function calculateComprehensiveSimilarity(text1, text2, options = {}) {
  const { cosineWeight = 0.5, jaccardWeight = 0.5 } = options;

  const cosineScore = cosineSimilarity(text1, text2);
  const jaccardScore = jaccardSimilarity(text1, text2);

  // Normalize weights
  const totalWeight = cosineWeight + jaccardWeight;
  const normalizedCosineWeight = cosineWeight / totalWeight;
  const normalizedJaccardWeight = jaccardWeight / totalWeight;

  // Calculate weighted average
  return (cosineScore * normalizedCosineWeight) + (jaccardScore * normalizedJaccardWeight);
}

/**
 * Detects similarity between content blocks using multiple algorithms
 * @param {Array<string>} contentBlocks - Array of content blocks to compare
 * @param {number} threshold - Similarity threshold (default: 0.7)
 * @returns {Array<Object>} Array of similarity pairs with details
 */
function detectContentSimilarity(contentBlocks, threshold = 0.7) {
  const similarities = [];

  for (let i = 0; i < contentBlocks.length; i++) {
    for (let j = i + 1; j < contentBlocks.length; j++) {
      const similarity = calculateComprehensiveSimilarity(contentBlocks[i], contentBlocks[j]);

      if (similarity >= threshold) {
        similarities.push({
          index1: i,
          index2: j,
          content1: contentBlocks[i],
          content2: contentBlocks[j],
          similarity: similarity,
          cosine: cosineSimilarity(contentBlocks[i], contentBlocks[j]),
          jaccard: jaccardSimilarity(contentBlocks[i], contentBlocks[j])
        });
      }
    }
  }

  // Sort by similarity score, highest first
  similarities.sort((a, b) => b.similarity - a.similarity);

  return similarities;
}

/**
 * Extracts semantic features from content for similarity comparison
 * @param {string} content - Content to extract features from
 * @returns {Object} Object containing extracted features
 */
function extractSemanticFeatures(content) {
  const tokens = preprocessText(content);
  
  // Count of unique stemmed words
  const uniqueWords = new Set(tokens).size;
  
  // Average word length
  const avgWordLength = tokens.length > 0 
    ? tokens.reduce((sum, token) => sum + token.length, 0) / tokens.length 
    : 0;
  
  // Length of content in characters
  const contentLength = content.length;
  
  // Number of sentences (approximate)
  const sentenceCount = content.split(/[.!?]+/).filter(s => s.trim().length > 0).length;
  
  return {
    uniqueWords,
    avgWordLength,
    contentLength,
    sentenceCount
  };
}

module.exports = {
  cosineSimilarity,
  jaccardSimilarity,
  calculateComprehensiveSimilarity,
  detectContentSimilarity,
  extractSemanticFeatures
};