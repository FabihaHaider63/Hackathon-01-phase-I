# AI-Driven Book on ROS 2

This repository contains an educational book on Robot Operating System 2 (ROS 2) for humanoid robotics, created using Docusaurus and AI-assisted content generation.

## Features

- Educational content on ROS 2 fundamentals for humanoid robotics
- AI-assisted content generation using Claude Code
- Structured learning modules with practical examples
- Clean, accessible design with consistent color scheme (white background, black text, #80303F accents)

## Prerequisites

- Node.js 18+ 
- npm or yarn package manager
- Claude Code API key (for content generation)

## Installation

1. Clone this repository
2. Install dependencies:
   ```bash
   npm install
   ```
3. Start the development server:
   ```bash
   npm start
   ```

## Project Structure

- `/docs` - Contains the book content in Markdown format
- `/src` - Contains custom React components and CSS
- `/static` - Contains static assets
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration

## Development

To add new content:
1. Create a new Markdown file in the appropriate module directory under `/docs`
2. Add proper frontmatter with title, sidebar_position, and description
3. Update `sidebars.js` if necessary to include the new content in navigation

## Deployment

This site is configured for deployment to GitHub Pages. The deployment process will:
1. Build the Docusaurus site
2. Deploy to the configured GitHub Pages branch

## Contributing

This book is continuously evolving. If you find errors or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License.