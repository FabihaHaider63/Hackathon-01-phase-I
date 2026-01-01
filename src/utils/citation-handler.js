/**
 * Citation Handler Utility
 * Manages citations in generated content following academic standards
 */

/**
 * Validates a citation object
 * @param {Object} citation - The citation object to validate
 * @returns {Object} Validation result with issues and status
 */
function validateCitation(citation) {
  const issues = [];
  
  // Required fields for different citation types
  const requiredFields = {
    'journal': ['author', 'title', 'journal', 'year', 'volume', 'pages'],
    'book': ['author', 'title', 'publisher', 'year'],
    'conference': ['author', 'title', 'booktitle', 'year', 'pages'],
    'web': ['title', 'url', 'accessed'],
    'thesis': ['author', 'title', 'school', 'year'],
    'report': ['author', 'title', 'institution', 'year'],
    'unpublished': ['author', 'title', 'note']
  };
  
  const type = citation.type || 'journal'; // Default type
  const required = requiredFields[type] || requiredFields['journal'];
  
  // Check for required fields
  for (const field of required) {
    if (!citation[field] || citation[field].toString().trim() === '') {
      issues.push(`Missing required field: ${field} for citation type: ${type}`);
    }
  }
  
  // Validate specific field formats
  if (citation.year) {
    const year = parseInt(citation.year);
    const currentYear = new Date().getFullYear();
    if (isNaN(year) || year < 1900 || year > currentYear + 1) {
      issues.push(`Year ${citation.year} is invalid or out of range`);
    }
  }
  
  if (citation.url && !isValidUrl(citation.url)) {
    issues.push(`URL ${citation.url} is not a valid URL format`);
  }
  
  return {
    isValid: issues.length === 0,
    issues: issues,
    citation: citation
  };
}

/**
 * Checks if a string is a valid URL
 * @param {string} url - The URL to validate
 * @returns {boolean} True if the URL is valid, false otherwise
 */
function isValidUrl(url) {
  try {
    new URL(url);
    return true;
  } catch (err) {
    return false;
  }
}

/**
 * Formats a citation in APA style
 * @param {Object} citation - The citation object
 * @returns {string} The formatted citation
 */
function formatCitationAPA(citation) {
  let result = '';
  
  // Author(s)
  if (citation.author) {
    if (Array.isArray(citation.author)) {
      result += citation.author.map((author, index) => {
        if (index === 0) {
          // First author: Last name, First initial.
          return formatAuthorName(author, true);
        } else {
          // Additional authors: Last name, First initial.
          return formatAuthorName(author, true);
        }
      }).join(', ');
      result += '. ';
    } else {
      result += formatAuthorName(citation.author, true) + '. ';
    }
  }
  
  // Year
  if (citation.year) {
    result += `(${citation.year}). `;
  }
  
  // Title
  if (citation.title) {
    result += capitalizeTitle(citation.title) + '. ';
  }
  
  // Additional fields based on type
  switch (citation.type) {
    case 'journal':
      // Journal, Volume(Issue), pages
      if (citation.journal) result += citation.journal + ', ';
      if (citation.volume) result += citation.volume;
      if (citation.issue) result += `(${citation.issue})`;
      if (citation.pages) result += `, ${citation.pages}.`;
      if (citation.doi) result += ` https://doi.org/${citation.doi}`;
      break;
      
    case 'book':
      // Publisher
      if (citation.publisher) result += citation.publisher;
      if (citation.location) result += `: ${citation.location}`;
      result += '.';
      break;
      
    case 'conference':
      // In Conference Proceedings
      result += `In ${citation.booktitle} `;
      if (citation.pages) result += ` (pp. ${citation.pages})`;
      result += '.';
      break;
      
    case 'web':
      // URL and access date
      if (citation.url) result += citation.url + ' ';
      if (citation.accessed) {
        const accessDate = formatDateAccessed(citation.accessed);
        result += `(Accessed: ${accessDate})`;
      }
      result += '.';
      break;
      
    default:
      // General format
      result += citation.publisher ? citation.publisher + '. ' : '';
      if (citation.url) result += citation.url + ' ';
      if (citation.accessed) {
        const accessDate = formatDateAccessed(citation.accessed);
        result += `(Accessed: ${accessDate})`;
      }
  }
  
  return result.trim();
}

/**
 * Formats author name for APA style
 * @param {string|Object} author - The author name or object
 * @param {boolean} fullLastName - Whether to include full last name
 * @returns {string} Formatted author name
 */
function formatAuthorName(author, fullLastName = true) {
  if (typeof author === 'string') {
    // If author is a simple string like "John Doe"
    const parts = author.trim().split(/\s+/);
    if (parts.length === 1) {
      return parts[0];
    } else if (parts.length === 2) {
      return `${fullLastName ? parts[1] : parts[1].charAt(0).toUpperCase()}. ${parts[0].charAt(0).toUpperCase()}.`;
    } else {
      // Multiple names, last name first, others as initials
      const lastName = parts.pop();
      const initials = parts.map(name => `${name.charAt(0).toUpperCase()}.`).join(' ');
      return `${fullLastName ? lastName : lastName.charAt(0).toUpperCase()}. ${initials}`;
    }
  } else if (typeof author === 'object') {
    // If author is an object like {first: "John", last: "Doe"}
    return `${author.last ? (fullLastName ? author.last : author.last.charAt(0).toUpperCase() + '.') : ''}, ${author.first ? author.first.charAt(0).toUpperCase() + '.' : ''}`;
  }
  
  return author;
}

/**
 * Capitalizes the title following APA rules
 * @param {string} title - The title to capitalize
 * @returns {string} Capitalized title
 */
function capitalizeTitle(title) {
  if (!title) return '';
  
  // APA title case: capitalize first word, last word, and all important words
  const words = title.split(' ');
  const result = words.map((word, index) => {
    // Always capitalize first and last word
    if (index === 0 || index === words.length - 1) {
      return capitalizeFirstLetter(word);
    }
    
    // Don't capitalize articles, short prepositions, short conjunctions
    const lowercaseWords = ['a', 'an', 'the', 'and', 'but', 'or', 'for', 'nor', 'on', 'at', 'to', 'from', 'by', 'of', 'in'];
    if (lowercaseWords.includes(word.toLowerCase())) {
      return word.toLowerCase();
    }
    
    return capitalizeFirstLetter(word);
  });
  
  return result.join(' ');
}

/**
 * Capitalizes the first letter of a word
 * @param {string} word - The word to capitalize
 * @returns {string} Capitalized word
 */
function capitalizeFirstLetter(word) {
  if (!word) return word;
  return word.charAt(0).toUpperCase() + word.slice(1).toLowerCase();
}

/**
 * Formats the accessed date for web citations
 * @param {string|Date} date - The date to format
 * @returns {string} Formatted date
 */
function formatDateAccessed(date) {
  if (!date) return '';
  
  const d = typeof date === 'string' ? new Date(date) : date;
  return d.toLocaleDateString('en-US', { 
    month: 'short', 
    day: 'numeric', 
    year: 'numeric' 
  });
}

/**
 * Formats a citation in the specified style
 * @param {Object} citation - The citation to format
 * @param {string} style - The citation style (apa, mla, chicago, etc.)
 * @returns {string} The formatted citation
 */
function formatCitation(citation, style = 'apa') {
  switch (style.toLowerCase()) {
    case 'apa':
      return formatCitationAPA(citation);
    case 'mla':
      return formatCitationMLA(citation);
    case 'chicago':
      return formatCitationChicago(citation);
    default:
      return formatCitationAPA(citation); // Default to APA
  }
}

/**
 * Formats a citation in MLA style
 * @param {Object} citation - The citation to format
 * @returns {string} The formatted citation
 */
function formatCitationMLA(citation) {
  let result = '';
  
  // Author(s)
  if (citation.author) {
    if (Array.isArray(citation.author)) {
      result += citation.author.map((author, index) => {
        if (index === 0) {
          return formatAuthorNameMLA(author, true);
        } else {
          // Additional authors: First initial. Last name
          return formatAuthorNameMLA(author, false);
        }
      }).join(', ').replace(',', ',');
      result += '. ';
    } else {
      result += formatAuthorNameMLA(citation.author, true) + '. ';
    }
  }
  
  // Title
  if (citation.title) {
    result += `"${capitalizeTitle(citation.title)}." `;
  }
  
  // Additional fields based on type
  switch (citation.type) {
    case 'journal':
      if (citation.journal) result += citation.journal + ', ';
      if (citation.volume) result += citation.volume + ', ';
      if (citation.issue) result += citation.issue + ', ';
      if (citation.year) result += citation.year + ', ';
      if (citation.pages) result += citation.pages + '.';
      break;
      
    case 'book':
      if (citation.publisher) result += citation.publisher + ', ';
      if (citation.year) result += citation.year + '.';
      break;
      
    case 'web':
      if (citation.url) result += citation.url + '. ';
      if (citation.accessed) {
        const accessDate = formatDateAccessed(citation.accessed);
        result += accessDate + '.';
      }
      break;
  }
  
  return result.trim();
}

/**
 * Formats author name for MLA style
 * @param {string|Object} author - The author name or object
 * @param {boolean} firstAuthor - Whether this is the first author
 * @returns {string} Formatted author name
 */
function formatAuthorNameMLA(author, firstAuthor = true) {
  if (typeof author === 'string') {
    const parts = author.trim().split(/\s+/);
    if (parts.length === 2) {
      if (firstAuthor) {
        return `${parts[1]}, ${parts[0]}`;
      } else {
        return `${parts[0]} ${parts[1]}`;
      }
    }
    return author;
  } else if (typeof author === 'object') {
    if (firstAuthor) {
      return `${author.last ? author.last : ''}, ${author.first ? author.first : ''}`;
    } else {
      return `${author.first ? author.first : ''} ${author.last ? author.last : ''}`;
    }
  }
  
  return author;
}

/**
 * Formats a citation in Chicago style
 * @param {Object} citation - The citation to format
 * @returns {string} The formatted citation
 */
function formatCitationChicago(citation) {
  // Chicago style formatting
  let result = '';
  
  if (citation.author) {
    if (Array.isArray(citation.author)) {
      result += citation.author.map((author, index) => {
        if (index === 0) {
          return formatAuthorNameChicago(author, true);
        } else {
          return formatAuthorNameChicago(author, false);
        }
      }).join(', ');
      result += '. ';
    } else {
      result += formatAuthorNameChicago(citation.author, true) + '. ';
    }
  }
  
  if (citation.year) result += citation.year + '. ';
  if (citation.title) result += `"${capitalizeTitle(citation.title)}." `;
  
  switch (citation.type) {
    case 'journal':
      if (citation.journal) result += citation.journal + ' ';
      if (citation.volume) result += citation.volume + ', no. ';
      if (citation.issue) result += citation.issue + ' ';
      if (citation.year) result += `(${citation.year})`;
      if (citation.pages) result += ': ' + citation.pages + '.';
      break;
      
    case 'book':
      if (citation.publisher) result += citation.publisher + ', ';
      if (citation.year) result += citation.year + '.';
      break;
  }
  
  return result.trim();
}

/**
 * Formats author name for Chicago style
 * @param {string|Object} author - The author name or object
 * @param {boolean} firstAuthor - Whether this is the first author
 * @returns {string} Formatted author name
 */
function formatAuthorNameChicago(author, firstAuthor = true) {
  if (typeof author === 'string') {
    const parts = author.trim().split(/\s+/);
    if (parts.length === 2) {
      if (firstAuthor) {
        return `${parts[1]}, ${parts[0]}`;
      } else {
        return `${parts[0]} ${parts[1]}`;
      }
    }
    return author;
  }
  
  return author;
}

/**
 * Extracts citations from content text
 * @param {string} content - The content to extract citations from
 * @returns {Array} Array of citation objects found in the content
 */
function extractCitations(content) {
  const citations = [];
  
  // Look for citation patterns in the text
  const apaPattern = /\(([A-Z][a-zA-Z]+, \d{4}[a-z]*)\)/g;
  const numberedPattern = /\[[\d,\-\s]+\]/g;  // [1], [2, 3], [4-6]
  const footnotePattern = /\[\d+\]/g;  // [1], [2], etc.
  
  let matches;
  
  // Find APA-style in-text citations
  while ((matches = apaPattern.exec(content)) !== null) {
    citations.push({
      text: matches[0],
      type: 'in-text',
      style: 'apa',
      position: matches.index
    });
  }
  
  // Find numbered citations
  while ((matches = numberedPattern.exec(content)) !== null) {
    citations.push({
      text: matches[0],
      type: 'numbered',
      style: 'ieee',
      position: matches.index
    });
  }
  
  return citations;
}

/**
 * Adds citations to content and creates a bibliography
 * @param {string} content - The content to add citations to
 * @param {Array} citations - Array of citation objects
 * @returns {Object} Object containing updated content and bibliography
 */
function addCitationsToContent(content, citations) {
  // This is a simplified implementation
  // In a real implementation, we would replace citation placeholders with
  // proper citations and format a bibliography
  
  let updatedContent = content;
  const bibliography = [];
  
  citations.forEach((citation, index) => {
    // Validate the citation
    const validation = validateCitation(citation);
    if (!validation.isValid) {
      console.warn(`Invalid citation at index ${index}:`, validation.issues);
      return;
    }
    
    // Add citation to bibliography
    const formattedCitation = formatCitation(citation, citation.style || 'apa');
    bibliography.push({
      id: citation.id || `ref${index + 1}`,
      formatted: formattedCitation,
      original: citation
    });
    
    // In a real implementation, we would replace citation placeholders
    // like [SOURCE1] with proper citation callouts such as [1] or (Author, 2023)
    if (citation.id) {
      updatedContent = updatedContent.replace(
        new RegExp(`\\[${citation.id}\\]`, 'g'),
        `[${index + 1}]`
      );
    }
  });
  
  return {
    content: updatedContent,
    bibliography: bibliography,
    citationCount: bibliography.length
  };
}

/**
 * Creates a bibliography section in Markdown format
 * @param {Array} bibliography - Array of formatted citation objects
 * @returns {string} Bibliography in Markdown format
 */
function createBibliographyMarkdown(bibliography) {
  if (!bibliography || bibliography.length === 0) {
    return '';
  }
  
  let bibText = '\n## References\n\n';
  
  bibliography.forEach((entry, index) => {
    bibText += `${index + 1}. ${entry.formatted}\n\n`;
  });
  
  return bibText;
}

module.exports = {
  validateCitation,
  formatCitation,
  formatCitationAPA,
  formatCitationMLA,
  formatCitationChicago,
  extractCitations,
  addCitationsToContent,
  createBibliographyMarkdown,
  isValidUrl
};