/**
 * Code Example Validation Utility for ROS 2 Snippets
 * Validates code examples to ensure they follow ROS 2 best practices and are functionally correct
 */

const fs = require('fs').promises;

/**
 * Validates Python code snippets for ROS 2 best practices
 * @param {string} code - The Python code to validate
 * @returns {Object} Validation result with issues and recommendations
 */
function validatePythonROS2Code(code) {
  const issues = [];
  const recommendations = [];
  
  // Check for proper node initialization
  if (!code.includes('rclpy.init') || !code.includes('rclpy.shutdown')) {
    issues.push('Missing proper rclpy initialization/shutdown');
    recommendations.push('Always call rclpy.init(args=args) at the start and rclpy.shutdown() at the end');
  }
  
  // Check for proper node inheritance
  if (code.includes('Node') && !code.includes('from rclpy.node import Node')) {
    issues.push('Node class used without proper import');
    recommendations.push('Import Node from rclpy.node: from rclpy.node import Node');
  }
  
  // Check for proper spin usage
  if (code.includes('rclpy.spin') && !code.includes('try:') && !code.includes('KeyboardInterrupt')) {
    issues.push('Missing exception handling for rclpy.spin()');
    recommendations.push('Wrap rclpy.spin() in try/except block to handle KeyboardInterrupt gracefully');
  }
  
  // Check for proper destruction
  if (code.includes('class') && code.includes('Node') && !code.includes('destroy_node')) {
    issues.push('Missing node destruction in cleanup');
    recommendations.push('Call self.destroy_node() in the cleanup routine');
  }
  
  // Check for proper parameter declaration
  if (code.includes('declare_parameter') && !code.includes('get_parameter')) {
    issues.push('Parameter declared but not accessed');
    recommendations.push('Use get_parameter() to access declared parameters');
  }
  
  // Check for proper QoS usage
  if (code.includes('QoSProfile') && !code.includes('ReliabilityPolicy') && !code.includes('DurabilityPolicy')) {
    recommendations.push('Consider using appropriate QoS policies for your use case');
  }
  
  // Check for common ROS 2 Python patterns
  const commonROS2Imports = [
    'from rclpy.node import Node',
    'import rclpy',
    'from std_msgs.msg import',
    'from sensor_msgs.msg import'
  ];
  
  const hasROS2Import = commonROS2Imports.some(imp => code.includes(imp));
  if (!hasROS2Import && (code.includes('Node') || code.includes('rclpy') || code.includes('msg'))) {
    issues.push('Missing common ROS 2 imports');
    recommendations.push('Ensure proper imports for ROS 2 functionality');
  }
  
  // Check for main function pattern
  if (code.includes('def main') && !code.includes("if __name__ == '__main__':")) {
    issues.push('Missing main execution pattern');
    recommendations.push('Use if __name__ == \'__main__\': pattern for proper execution');
  }
  
  return {
    isValid: issues.length === 0,
    issues,
    recommendations,
    score: Math.max(0, 100 - (issues.length * 10)) // Simple scoring system
  };
}

/**
 * Validates C++ code snippets for ROS 2 best practices
 * @param {string} code - The C++ code to validate
 * @returns {Object} Validation result with issues and recommendations
 */
function validateCppROS2Code(code) {
  const issues = [];
  const recommendations = [];
  
  // Check for proper initialization
  if (!code.includes('rclcpp::init') || !code.includes('rclcpp::shutdown')) {
    issues.push('Missing proper rclcpp initialization/shutdown');
    recommendations.push('Always call rclcpp::init() at the start and rclcpp::shutdown() at the end');
  }
  
  // Check for proper node creation
  const hasNodeCreation = code.includes('rclcpp::Node') || code.includes('make_shared') && code.includes('Node');
  if (!hasNodeCreation) {
    issues.push('No ROS 2 node creation detected');
    recommendations.push('Create a Node or inherit from rclcpp::Node');
  }
  
  // Check for proper spin usage
  if (code.includes('spin') && !code.includes('rclcpp::spin')) {
    issues.push('Incorrect spin usage');
    recommendations.push('Use rclcpp::spin(node) for spinning the node');
  }
  
  // Check for proper headers
  const ros2Headers = [
    '#include <rclcpp/rclcpp.hpp>',
    '#include <std_msgs/msg.hpp>',
    '#include <sensor_msgs/msg.hpp>'
  ];
  
  const hasROS2Header = ros2Headers.some(hdr => code.includes(hdr));
  if (!hasROS2Header && (code.includes('rclcpp') || code.includes('Node') || code.includes('msg'))) {
    issues.push('Missing ROS 2 headers');
    recommendations.push('Include necessary ROS 2 headers');
  }
  
  return {
    isValid: issues.length === 0,
    issues,
    recommendations,
    score: Math.max(0, 100 - (issues.length * 10))
  };
}

/**
 * Validates code for general correctness and best practices
 * @param {string} code - The code to validate
 * @param {string} language - The programming language ('python', 'cpp', etc.)
 * @returns {Object} Validation result
 */
function validateCodeExample(code, language = 'python') {
  // Basic code validation regardless of language
  const issues = [];
  const recommendations = [];
  
  // Check for common issues
  if (code.length < 10) {
    issues.push('Code example is too short to be meaningful');
    recommendations.push('Provide more complete code examples with proper structure');
  }
  
  if (code.includes('TODO:') || code.includes('// TODO') || code.includes('# TODO')) {
    issues.push('Code contains TODO placeholders');
    recommendations.push('Complete all TODOs before finalizing the example');
  }
  
  if (code.includes('pass') && language === 'python' && code.includes('def ')) {
    const functionCount = (code.match(/def\s+\w+\s*\(/g) || []).length;
    const passCount = (code.match(/\bpass\b/g) || []).length;
    if (functionCount > 0 && passCount >= functionCount) {
      issues.push('Functions contain only pass statements');
      recommendations.push('Implement the function logic instead of using pass');
    }
  }
  
  let langValidation = { isValid: true, issues: [], recommendations: [], score: 100 };
  
  if (language === 'python') {
    langValidation = validatePythonROS2Code(code);
  } else if (language === 'cpp') {
    langValidation = validateCppROS2Code(code);
  }
  
  // Combine general and language-specific validation
  const combinedIssues = [...issues, ...langValidation.issues];
  const combinedRecommendations = [...recommendations, ...langValidation.recommendations];
  
  return {
    isValid: langValidation.isValid && issues.length === 0,
    issues: combinedIssues,
    recommendations: combinedRecommendations,
    score: Math.min(langValidation.score, Math.max(0, 100 - (issues.length * 10))),
    languageSpecific: langValidation
  };
}

/**
 * Validates multiple code examples in a document
 * @param {string} documentContent - The document content containing code examples
 * @returns {Array} Array of validation results for each code block
 */
function validateDocumentCodeExamples(documentContent) {
  // Find all code blocks in the document
  const codeBlocks = documentContent.match(/```(\w+)?\n([\s\S]*?)\n```/g);
  const results = [];
  
  if (!codeBlocks) {
    return results;
  }
  
  for (const block of codeBlocks) {
    // Extract language and code
    const match = block.match(/```(\w+)?\n([\s\S]*?)\n```/);
    if (match) {
      const language = match[1] || 'python'; // default to python
      const code = match[2];
      
      const validation = validateCodeExample(code, language);
      results.push({
        code: code,
        language: language,
        validation: validation
      });
    }
  }
  
  return results;
}

/**
 * Generates a summary report of code validation results
 * @param {Array} validationResults - Array of validation results from validateDocumentCodeExamples
 * @returns {Object} Summary report
 */
function generateValidationReport(validationResults) {
  if (!validationResults || validationResults.length === 0) {
    return {
      totalCodeBlocks: 0,
      validCodeBlocks: 0,
      invalidCodeBlocks: 0,
      totalIssues: 0,
      avgScore: 0,
      summary: "No code blocks found to validate"
    };
  }
  
  const totalCodeBlocks = validationResults.length;
  const validCodeBlocks = validationResults.filter(result => result.validation.isValid).length;
  const invalidCodeBlocks = totalCodeBlocks - validCodeBlocks;
  const totalIssues = validationResults.reduce((sum, result) => sum + result.validation.issues.length, 0);
  const avgScore = validationResults.reduce((sum, result) => sum + result.validation.score, 0) / totalCodeBlocks;
  
  return {
    totalCodeBlocks,
    validCodeBlocks,
    invalidCodeBlocks,
    totalIssues,
    avgScore: parseFloat(avgScore.toFixed(2)),
    summary: `${validCodeBlocks}/${totalCodeBlocks} code blocks passed validation with an average score of ${avgScore.toFixed(2)}/100`
  };
}

module.exports = {
  validateCodeExample,
  validateDocumentCodeExamples,
  generateValidationReport,
  validatePythonROS2Code,
  validateCppROS2Code
};