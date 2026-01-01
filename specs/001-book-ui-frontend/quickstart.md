# Quickstart: Book UI Frontend

## Overview
This guide provides instructions for setting up and running the Book UI Frontend feature.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Basic knowledge of React and TypeScript

## Setup Instructions

### 1. Install Dependencies
```bash
npm install react react-dom @types/react @types/react-dom
npm install tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

### 2. Configure Tailwind CSS
Update your `tailwind.config.js`:
```js
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {
      borderRadius: {
        'xl': '0.75rem',
        '2xl': '1rem',
      },
      boxShadow: {
        'sm': '0 1px 2px 0 rgba(0, 0, 0, 0.05)',
        'md': '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)',
      }
    },
  },
  plugins: [],
}
```

### 3. Add Tailwind Directives
In your main CSS file (e.g., `src/index.css`):
```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

## Running the Application

### Development Mode
```bash
npm start
```

### Building for Production
```bash
npm run build
```

## Component Structure
The Book UI Frontend consists of the following components:

### BookPage (Main Component)
The main page component that orchestrates the layout and data flow.

### BookOverviewBox
Displays the book summary, key modules, audience, and difficulty level.

### AuthorBox
Displays the author's name, role, and professional bio.

### FooterLinks
Displays the required professional links with appropriate alignment.

## Data Flow
1. BookPage receives book, author, and footer link data as props
2. BookPage renders BookOverviewBox and AuthorBox side-by-side (desktop) or stacked (mobile)
3. FooterLinks renders at the bottom with proper alignment based on screen size
4. Components use Tailwind classes for responsive styling

## Customization
To customize the book content:
1. Update the data passed to the BookPage component
2. Modify the content in the data model as needed
3. Adjust styling through Tailwind classes if needed

## Testing
Run unit tests:
```bash
npm test
```

Run E2E tests:
```bash
npm run e2e
```