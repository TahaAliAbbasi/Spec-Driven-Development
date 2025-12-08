# Card Component Documentation

## Overview
The Card component is a reusable React component that displays a card with an image, title, description, and a "Read More" button for navigation. It is used to create the homepage cards grid that links to documentation pages.

## Location
- **Component**: `Physical-AI-and-Humanoid-Robotics/src/components/Card/index.tsx`
- **Styles**: `Physical-AI-and-Humanoid-Robotics/src/components/Card/styles.module.css`
- **Grid Component**: `Physical-AI-and-Humanoid-Robotics/src/components/CardsGrid/index.tsx`
- **Data Configuration**: `Physical-AI-and-Humanoid-Robotics/src/data/homepage-cards.json`

## Component API

### Card Component Props
| Prop | Type | Required | Description |
|------|------|----------|-------------|
| `title` | string | Yes | The title displayed on the card |
| `description` | string | Yes | The short description text shown on the card |
| `imageUrl` | string | Yes | Path to the image asset to display on the card |
| `link` | string | Yes | URL to navigate to when "Read More" button is clicked |
| `className` | string | No | Additional CSS classes to apply to the card |

### Usage Example
```tsx
import Card from '@site/src/components/Card';

<Card
  title="Module 1"
  description="Introduction to Physical AI and Humanoid Robotics"
  imageUrl="/img/module1-icon.png"
  link="/docs/module1/"
  className="custom-card-class"
/>
```

## CardsGrid Component
The CardsGrid component renders a grid of Card components based on the data from the configuration file.

### Usage
```tsx
import CardsGrid from '@site/src/components/CardsGrid';

<CardsGrid />
```


## Data Configuration
Card content is managed through the `homepage-cards.json` file. See `src/data/README.md` for detailed information about the configuration format.

## Features
- Responsive design that adapts to mobile, tablet, and desktop screens
- Lazy loading for images to improve performance
- Fallback images for missing image assets
- Accessibility features including ARIA labels and semantic HTML
- Hover effects for improved user experience
- Data validation to ensure all required properties are present

## Maintenance
To update card content:
1. Modify the `src/data/homepage-cards.json` file
2. Ensure all required properties are present
3. Verify image paths and documentation links are valid
4. Test the changes by running the development server

## Troubleshooting
- If cards don't appear: Check that `homepage-cards.json` is properly formatted and contains valid data
- If images don't load: Verify image paths are correct and files exist in the static directory
- If links don't work: Confirm documentation paths are valid and pages exist