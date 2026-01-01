/**
 * Readability Validator Utility
 * Validates content meets Flesch-Kincaid Grade Level requirements (10-12)
 */

/**
 * Counts syllables in a word using a simple algorithm
 * @param {string} word - The word to count syllables for
 * @returns {number} The number of syllables in the word
 */
function countSyllables(word) {
  // Convert to lowercase and remove non-alphabetic characters
  word = word.toLowerCase().replace(/[^a-z]/g, '');
  
  if (word.length <= 3) {
    return 1;
  }
  
  // Count vowel groups
  const vowels = word.match(/[aeiouy]+/g);
  let syllableCount = vowels ? vowels.length : 0;
  
  // Subtract 1 for silent 'e' at the end
  if (word.endsWith('e') && syllableCount > 1) {
    syllableCount--;
  }
  
  // Add 1 if word ends with 'le' preceded by a consonant
  if (word.endsWith('le') && word.length > 2 && !'aeiouy'.includes(word.charAt(word.length - 3))) {
    syllableCount++;
  }
  
  // Ensure at least 1 syllable
  return Math.max(1, syllableCount);
}

/**
 * Calculates the Flesch-Kincaid Grade Level for given text
 * @param {string} text - The text to analyze
 * @returns {number} The Flesch-Kincaid Grade Level
 */
function calculateFleschKincaidGradeLevel(text) {
  // Remove HTML tags if present and normalize whitespace
  const cleanText = text.replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ').trim();
  
  if (!cleanText) {
    return 0; // Return 0 for empty text
  }
  
  // Count words
  const words = cleanText.match(/\b\w+\b/g) || [];
  const wordCount = words.length;
  
  if (wordCount === 0) {
    return 0;
  }
  
  // Count sentences (ending with ., !, or ?)
  const sentences = cleanText.match(/[.!?]+/g) || [];
  const sentenceCount = sentences.length;
  
  if (sentenceCount === 0) {
    // If no sentences, return basic readability based on word count
    return Math.min(15, Math.floor(wordCount / 50)); // Estimate for fragment
  }
  
  // Count syllables
  let syllableCount = 0;
  for (const word of words) {
    syllableCount += countSyllables(word);
  }
  
  // Calculate average words per sentence and syllables per word
  const avgWordsPerSentence = wordCount / sentenceCount;
  const avgSyllablesPerWord = syllableCount / wordCount;
  
  // Flesch-Kincaid Grade Level formula
  // 0.39 * (words/sentences) + 11.8 * (syllables/word) - 15.59
  const gradeLevel = (0.39 * avgWordsPerSentence) + (11.8 * avgSyllablesPerWord) - 15.59;
  
  return Math.max(0, parseFloat(gradeLevel.toFixed(2)));
}

/**
 * Validates that content meets readability requirements
 * @param {string} content - The content to validate
 * @param {number} minGradeLevel - Minimum grade level (default: 10)
 * @param {number} maxGradeLevel - Maximum grade level (default: 12)
 * @returns {Object} Validation result with score and pass/fail status
 */
function validateReadability(content, minGradeLevel = 10, maxGradeLevel = 12) {
  const gradeLevel = calculateFleschKincaidGradeLevel(content);
  
  return {
    gradeLevel: gradeLevel,
    isValid: gradeLevel >= minGradeLevel && gradeLevel <= maxGradeLevel,
    message: gradeLevel < minGradeLevel 
      ? `Content is too easy (grade level: ${gradeLevel.toFixed(1)}). Target: ${minGradeLevel}-${maxGradeLevel}.`
      : gradeLevel > maxGradeLevel
        ? `Content is too difficult (grade level: ${gradeLevel.toFixed(1)}). Target: ${minGradeLevel}-${maxGradeLevel}.`
        : `Content readability is appropriate (grade level: ${gradeLevel.toFixed(1)}).`,
    recommendations: generateReadabilityRecommendations(content, gradeLevel, minGradeLevel, maxGradeLevel)
  };
}

/**
 * Generates recommendations to improve readability
 * @param {string} content - The content to analyze
 * @param {number} currentGradeLevel - Current grade level of the content
 * @param {number} minGradeLevel - Minimum acceptable grade level
 * @param {number} maxGradeLevel - Maximum acceptable grade level
 * @returns {Array<string>} Recommendations to improve readability
 */
function generateReadabilityRecommendations(content, currentGradeLevel, minGradeLevel, maxGradeLevel) {
  const recommendations = [];
  
  if (currentGradeLevel > maxGradeLevel) {
    // Content is too complex - suggest simplifications
    recommendations.push('Consider using shorter sentences to improve readability.');
    recommendations.push('Replace complex words with simpler alternatives where possible.');
    recommendations.push('Break down complex concepts into smaller, digestible sections.');
  } else if (currentGradeLevel < minGradeLevel) {
    // Content is too simple - suggest enhancements
    recommendations.push('Consider adding more technical depth to the content.');
    recommendations.push('Include more detailed explanations of complex topics.');
  }
  
  return recommendations;
}

/**
 * Calculates various readability metrics for the given text
 * @param {string} content - The content to analyze
 * @returns {Object} Object containing various readability metrics
 */
function calculateReadabilityMetrics(content) {
  const cleanText = content.replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ').trim();
  const words = cleanText.match(/\b\w+\b/g) || [];
  const sentences = cleanText.match(/[.!?]+/g) || [];
  const syllables = words.map(word => countSyllables(word));
  
  const totalWords = words.length;
  const totalSentences = sentences.length;
  const totalSyllables = syllables.reduce((sum, count) => sum + count, 0);
  const avgSyllablesPerWord = totalWords > 0 ? totalSyllables / totalWords : 0;
  const avgWordsPerSentence = totalSentences > 0 ? totalWords / totalSentences : 0;
  
  return {
    wordCount: totalWords,
    sentenceCount: totalSentences,
    syllableCount: totalSyllables,
    avgSyllablesPerWord: parseFloat(avgSyllablesPerWord.toFixed(2)),
    avgWordsPerSentence: parseFloat(avgWordsPerSentence.toFixed(2)),
    fleschKincaidGradeLevel: calculateFleschKincaidGradeLevel(content)
  };
}

module.exports = {
  validateReadability,
  calculateFleschKincaidGradeLevel,
  calculateReadabilityMetrics,
  countSyllables
};