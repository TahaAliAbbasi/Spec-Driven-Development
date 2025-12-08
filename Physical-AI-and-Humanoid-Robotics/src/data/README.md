# Card Data Configuration Format

## Overview
The card data configuration file defines the content displayed in the homepage cards grid. This file is located at `src/data/homepage-cards.json` and follows a specific JSON structure.

## File Location
- **Path**: `Physical-AI-and-Humanoid-Robotics/src/data/homepage-cards.json`
- **Format**: JSON array of card objects
- **Purpose**: Define card content without code changes

## Card Object Structure

Each card in the array must have the following properties:

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| `id` | string | Yes | Unique identifier for the card (e.g., "module1-card") |
| `title` | string | Yes | Display title shown on the card |
| `description` | string | Yes | Short description text shown on the card |
| `imageUrl` | string | Yes | Path to the image asset (e.g., "/img/module1-icon.png") |
| `link` | string | Yes | URL to navigate to when "Read More" is clicked |
| `order` | number | Yes | Determines display order (lower numbers appear first) |

## Example Configuration

```json
[
  {
    "id": "module1-card",
    "title": "Module 1",
    "description": "Introduction to Physical AI and Humanoid Robotics - Module 1",
    "imageUrl": "/img/system-architecture.png",
    "link": "/docs/module1/chapter2",
    "order": 1
  }
]
```

## Validation Rules
- All properties are required and must have valid values
- The `id` must be unique across all cards
- The `imageUrl` should point to valid static assets in the `/img/` directory
- The `link` should be a valid internal path starting with `/docs/` or external URL
- The `order` values must be unique and sequential for proper display order

## Adding New Cards
1. Add a new card object to the JSON array
2. Ensure the `id` is unique
3. Verify the `order` value places the card in the desired position
4. Confirm the `imageUrl` points to an existing image file
5. Validate that the `link` points to an existing documentation page

## Updating Existing Cards
- Edit the appropriate card object in the JSON array
- Changes take effect immediately after redeploying the site