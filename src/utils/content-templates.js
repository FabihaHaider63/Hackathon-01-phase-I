/**
 * Content Template Structures for Consistent Chapter Formatting
 */

/**
 * Creates a standard chapter template with proper frontmatter
 * @param {string} title - The title of the chapter
 * @param {number} sidebarPosition - The position in the sidebar
 * @param {string} description - A brief description of the chapter content
 * @returns {string} The formatted chapter template
 */
function createChapterTemplate(title, sidebarPosition, description) {
  return `---
title: ${title}
sidebar_position: ${sidebarPosition}
description: ${description}
---

# ${title}

## Overview

This chapter provides an overview of the concepts and practices related to this topic in the context of AI-driven robotics and ROS 2.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Understand the fundamental concepts
- [ ] Apply the concepts in practical scenarios
- [ ] Integrate these concepts with other system components

## Main Content

### Section 1

[Content for the first section]

### Section 2

[Content for the second section]

## Code Examples

Here's a typical code example for this topic:

\`\`\`python
# Example code demonstrating the concepts
def example_function():
    print("This is a sample implementation")
    return True
\`\`\`

## Summary

[Summary of the key points covered in this chapter]

## Exercises

1. [Exercise 1]
2. [Exercise 2]

## Further Reading

- [Reference 1]
- [Reference 2]
`;
}

/**
 * Creates a standard module template
 * @param {string} title - The title of the module
 * @param {string} description - A brief description of the module
 * @returns {string} The formatted module template
 */
function createModuleTemplate(title, description) {
  return `---
title: ${title}
sidebar_position: 1
description: ${description}
---

# ${title}

## Module Overview

[Overview of the module content and objectives]

## Learning Outcomes

After completing this module, you will be able to:

- [ ] Understand the core concepts
- [ ] Implement solutions using these concepts
- [ ] Apply best practices in your projects

## Chapters

- [Chapter 1 Name](./chapter-1.md)
- [Chapter 2 Name](./chapter-2.md)
- [Chapter 3 Name](./chapter-3.md)

## Prerequisites

- [Prerequisite 1]
- [Prerequisite 2]

## Next Steps

[What to do after completing this module]
`;
}

/**
 * Creates a standard concept explanation template
 * @param {string} conceptName - The name of the concept
 * @param {string} definition - A clear definition of the concept
 * @returns {string} The formatted concept template
 */
function createConceptTemplate(conceptName, definition) {
  return `## ${conceptName}

### Definition

${definition}

### Why It Matters

[Explanation of why this concept is important in the context of AI-driven robotics and ROS 2]

### Real-World Applications

- [Application 1]
- [Application 2]
- [Application 3]

### Related Concepts

- [Related concept 1]
- [Related concept 2]

### Implementation

\`\`\`python
# Example implementation of ${conceptName}
def ${conceptName.toLowerCase().replace(/\s+/g, '_')}_implementation():
    # Implementation code here
    pass
\`\`\`
`;
}

/**
 * Creates a standard code example template
 * @param {string} title - The title of the code example
 * @param {string} description - A brief description
 * @param {string} language - The programming language (default: python)
 * @returns {string} The formatted code example template
 */
function createCodeExampleTemplate(title, description, language = 'python') {
  return `### ${title}

${description}

\`\`\`${language}
# ${title}
# ${description}

def example_function():
    # Implementation code
    result = "Implementation of the example"
    return result

# Usage example
if __name__ == "__main__":
    output = example_function()
    print(output)
\`\`\`

### Key Points

- [Key point 1]
- [Key point 2]
- [Key point 3]
`;
}

/**
 * Creates a standard summary template for chapters
 * @param {string} chapterTitle - Title of the chapter to summarize
 * @returns {string} The formatted summary template
 */
function createSummaryTemplate(chapterTitle) {
  return `## Summary

In this chapter on ${chapterTitle}, we covered:

1. [Main point 1]
2. [Main point 2]
3. [Main point 3]

### Key Takeaways

- [Takeaway 1]
- [Takeaway 2]
- [Takeaway 3]

### What's Next

[Explanation of how this chapter connects to the next chapter or broader concepts]
`;
}

module.exports = {
  createChapterTemplate,
  createModuleTemplate,
  createConceptTemplate,
  createCodeExampleTemplate,
  createSummaryTemplate
};