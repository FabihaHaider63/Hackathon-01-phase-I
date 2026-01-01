# Data Model: Book UI Frontend

## Overview
This document defines the data structures and models for the Book UI Frontend feature.

## Book Information Entity

**Description**: Contains the main information about the book to be displayed

**Fields**:
- `title` (string): The main title of the book
- `subtitle` (string): The subtitle of the book
- `summary` (string): A 2-4 line summary of the book content
- `keyModules` (string[]): List of key modules covered in the book
- `audience` (string): Target audience for the book
- `difficulty` (string): Difficulty level of the book (e.g., Beginner, Intermediate, Advanced)

**Validation Rules**:
- `title` must not be empty
- `summary` must be 2-4 lines (approximately 50-200 words)
- `keyModules` must contain at least one module
- `audience` must be provided
- `difficulty` must be one of the predefined levels

## Author Information Entity

**Description**: Contains information about the book's author

**Fields**:
- `name` (string): The author's name
- `role` (string): The author's role or title
- `bio` (string): A 2-line professional bio of the author
- `socialLinks` (object): Optional social media links for the author
  - `portfolio` (string): URL to portfolio
  - `linkedin` (string): URL to LinkedIn profile
  - `twitter` (string): URL to Twitter profile
  - `github` (string): URL to GitHub profile
  - `email` (string): Email address

**Validation Rules**:
- `name` must not be empty
- `role` must not be empty
- `bio` must be 2 lines maximum (approximately 20-50 words)
- URLs in `socialLinks` must be valid if provided

## Footer Links Entity

**Description**: Contains the specific links required in the footer

**Fields**:
- `portfolio` (string): URL to portfolio
- `linkedin` (string): URL to LinkedIn profile
- `twitter` (string): URL to Twitter profile
- `github` (string): URL to GitHub profile
- `email` (string): Email address or mailto link

**Validation Rules**:
- All links must be valid URLs
- At least one link must be provided

## Component State Models

### BookPage State
- `bookData` (BookInformation): The book information to display
- `authorData` (AuthorInformation): The author information to display
- `footerLinks` (FooterLinks): The links to display in the footer
- `isLoading` (boolean): Whether content is still loading
- `error` (string | null): Any error that occurred during loading

### Responsive Display State
- `isMobile` (boolean): Whether the current view is mobile-sized
- `isDesktop` (boolean): Whether the current view is desktop-sized