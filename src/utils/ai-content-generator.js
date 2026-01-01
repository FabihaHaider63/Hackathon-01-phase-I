/**
 * AI Content Generator Utility for Docusaurus Book
 * Uses Claude Code API for content generation
 */

const axios = require('axios');

/**
 * Generates content using Claude Code API
 * @param {Object} options - Configuration options for content generation
 * @param {string} options.topic - The topic to generate content about
 * @param {number} options.wordCount - Target word count for the content
 * @param {string} options.targetAudience - Target audience for the content
 * @param {boolean} options.includeCodeExamples - Whether to include code examples
 * @param {string} options.claudeApiKey - Claude API key
 * @returns {Promise<string>} Generated content
 */
async function generateContent(options) {
  const {
    topic,
    wordCount = 500,
    targetAudience = 'intermediate developers',
    includeCodeExamples = false,
    claudeApiKey
  } = options;

  // Validate required parameters
  if (!topic) {
    throw new Error('Topic is required for content generation');
  }

  if (!claudeApiKey) {
    throw new Error('Claude API key is required for content generation');
  }

  // Construct the prompt for Claude
  let prompt = `Generate educational content about "${topic}" for ${targetAudience}. `;
  prompt += `Target word count is approximately ${wordCount} words. `;
  prompt += `Ensure the content is clear, accurate, and follows educational best practices. `;

  if (includeCodeExamples) {
    prompt += `Include relevant code examples where appropriate, using proper syntax highlighting. `;
  }

  prompt += `Structure the content with appropriate headings and subheadings. `;

  // This is a placeholder implementation - in a real system, you would call the Claude API
  // For demonstration purposes, we'll return sample content
  console.log('Content generation request for topic:', topic);

  // In a production implementation, you would make an API call to Claude here:
  /*
  try {
    const response = await axios.post(
      'https://api.anthropic.com/v1/complete',
      {
        prompt: prompt,
        model: 'claude-3-opus-20240229', // or another Claude model
        max_tokens_to_sample: wordCount * 2, // roughly estimate tokens
        stop_sequences: ['\n\nHuman:'],
      },
      {
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': claudeApiKey,
        },
      }
    );

    return response.data.completion.trim();
  } catch (error) {
    console.error('Error generating content:', error.message);
    throw error;
  }
  */

  // For this demonstration, return sample content
  return generateSampleContent(topic, wordCount, includeCodeExamples);
}

/**
 * Generates sample content for demonstration purposes
 * @param {string} topic - The topic for the content
 * @param {number} wordCount - Target word count
 * @param {boolean} includeCodeExamples - Whether to include code examples
 * @returns {string} Sample content
 */
function generateSampleContent(topic, wordCount, includeCodeExamples) {
  const content = [];

  content.push(`# ${topic}`);
  content.push('');

  // Specific content generation based on topic
  if (topic.toLowerCase().includes('ros 2') || topic.toLowerCase().includes('robot operating system')) {
    content.push(`This chapter provides a comprehensive overview of the Robot Operating System 2 (ROS 2), the middleware that forms the nervous system of modern robotic applications. ROS 2 is designed to facilitate communication between different components of a robotic system, enabling complex behaviors through modular software packages.`);
    content.push('');
    content.push('## Introduction to ROS 2');
    content.push('');
    content.push('ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a collection of software frameworks and tools that help developers create robotic applications. It provides:');
    content.push('');
    content.push('- **Middleware Infrastructure**: A communication layer that allows different parts of a robotic system to interact seamlessly');
    content.push('- **Development Tools**: Utilities for debugging, visualizing, and testing robotic applications');
    content.push('- **Hardware Abstraction**: Interfaces that allow the same software to run on different hardware platforms');
    content.push('- **Package Management**: A system for organizing and distributing robotic software');
    content.push('');
    content.push('ROS 2 addresses limitations of the original ROS framework, particularly with real-time support, security, and multi-robot support.');

    if (includeCodeExamples) {
      content.push('');
      content.push('## Basic ROS 2 Example with rclpy');
      content.push('');
      content.push('Here is a simple ROS 2 Python node:');
      content.push('');
      content.push('```python');
      content.push('import rclpy');
      content.push('from rclpy.node import Node');
      content.push('');
      content.push('');
      content.push('class MinimalPublisher(Node):');
      content.push('');
      content.push('    def __init__(self):');
      content.push('        super().__init__(\'minimal_publisher\')');
      content.push('        self.publisher_ = self.create_publisher(String, \'topic\', 10)');
      content.push('        timer_period = 0.5  # seconds');
      content.push('        self.timer = self.create_timer(timer_period, self.timer_callback)');
      content.push('        self.i = 0');
      content.push('');
      content.push('    def timer_callback(self):');
      content.push('        msg = String()');
      content.push('        msg.data = \'Hello World: %d\' % self.i');
      content.push('        self.publisher_.publish(msg)');
      content.push('        self.get_logger().info(\'Publishing: "%s"\' % msg.data)');
      content.push('        self.i += 1');
      content.push('');
      content.push('');
      content.push('def main(args=None):');
      content.push('    rclpy.init(args=args)');
      content.push('');
      content.push('    minimal_publisher = MinimalPublisher()');
      content.push('');
      content.push('    rclpy.spin(minimal_publisher)');
      content.push('');
      content.push('    minimal_publisher.destroy_node()');
      content.push('    rclpy.shutdown()');
      content.push('');
      content.push('');
      content.push('if __name__ == \'__main__\':');
      content.push('    main()');
      content.push('```');
      content.push('');
    }

    content.push('## Key ROS 2 Concepts');
    content.push('');
    content.push('Understanding these core concepts is crucial for working with ROS 2:');
    content.push('');
    content.push('1. **Nodes**: Individual processes that perform computations. In a humanoid robot, these could be sensor nodes, control nodes, or AI decision-making nodes.');
    content.push('2. **Topics**: Named channels over which nodes exchange messages. These are used for continuous data streams like sensor data.');
    content.push('3. **Services**: Synchronous request/reply communication patterns. These are used for operations that require a specific response.');
    content.push('4. **Actions**: Asynchronous goal-oriented communication for long-running tasks with feedback.');
    content.push('5. **Packages**: Collections of related resources that implement specific functionality.');

    content.push('');
    content.push('## Quality of Service (QoS) in ROS 2');
    content.push('');
    content.push('ROS 2 introduces Quality of Service settings that allow fine-tuning communication behavior based on application requirements. These settings control reliability, durability, and other aspects of message delivery.');
  } else {
    content.push(`This is a sample chapter about ${topic}. In a real implementation, this content would be generated by the Claude Code API based on the provided parameters.`);

    content.push('');
    content.push('## Introduction');
    content.push('');
    content.push(`The fundamentals of ${topic} are essential for understanding how to develop robust and maintainable systems. This chapter covers key concepts and practical implementations.`);

    if (includeCodeExamples) {
      content.push('');
      content.push('## Code Example');
      content.push('');
      content.push('Here is a sample code example:');
      content.push('');
      content.push('```javascript');
      content.push('// Sample code snippet for demonstration');
      content.push('function sampleFunction() {');
      content.push('  console.log("This is a sample code example");');
      content.push('  return true;');
      content.push('}');
      content.push('```');
      content.push('');
    }

    content.push('## Key Concepts');
    content.push('');
    content.push('- Concept 1: Explanation of the first key concept');
    content.push('- Concept 2: Explanation of the second key concept');
    content.push('- Concept 3: Explanation of the third key concept');

    content.push('');
    content.push('## Summary');
    content.push('');
    content.push(`In this chapter, we covered the fundamentals of ${topic}. Understanding these concepts is crucial for advancing to more complex topics.`);
  }

  let result = content.join('\n');

  // Truncate or extend to approximate wordCount if needed for the demo
  const words = result.split(/\s+/);
  if (words.length > wordCount) {
    result = words.slice(0, wordCount).join(' ') + '...';
  } else if (words.length < wordCount) {
    // Add more content to reach the target word count
    while (words.length < wordCount) {
      result += '\n\nAdditional content to reach target word count. This section would contain more detailed information about the topic, including advanced concepts, practical examples, and implementation patterns relevant to the subject matter.';
      words.length = result.split(/\s+/).length;
    }
  }

  return result;
}

module.exports = {
  generateContent
};