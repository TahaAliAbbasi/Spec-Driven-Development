# Data Model: Homepage Cards

## Card Entity

**Attributes**:
- id: string (unique identifier for the card)
- title: string (display title of the module/capstone project)
- description: string (short description of the content)
- imageUrl: string (path to the image asset)
- link: string (URL to navigate to when "read more" is clicked)
- order: number (determines display order on the homepage)

## Card Data Structure

```json
[
  {
    "id": "module1-card",
    "title": "Module 1",
    "description": "Introduction to Physical AI and Humanoid Robotics - Module 1",
    "imageUrl": "/img/module1-icon.png",
    "link": "/docs/module1/chapter2",
    "order": 1
  },
  {
    "id": "module2-card",
    "title": "Module 2",
    "description": "Advanced topics in Physical AI and Humanoid Robotics - Module 2",
    "imageUrl": "/img/module2-icon.png",
    "link": "/docs/module2/",
    "order": 2
  },
  {
    "id": "module3-card",
    "title": "Module 3",
    "description": "Practical applications in Physical AI and Humanoid Robotics - Module 3",
    "imageUrl": "/img/module3-icon.png",
    "link": "/docs/module3/",
    "order": 3
  },
  {
    "id": "module4-card",
    "title": "Module 4",
    "description": "Advanced implementations in Physical AI and Humanoid Robotics - Module 4",
    "imageUrl": "/img/module4-icon.png",
    "link": "/docs/module4/",
    "order": 4
  },
  {
    "id": "capstone-card",
    "title": "Capstone Project",
    "description": "Comprehensive project integrating all modules of Physical AI and Humanoid Robotics",
    "imageUrl": "/img/capstone-icon.png",
    "link": "/docs/capstone/",
    "order": 5
  }
]
```

## Validation Rules

- Each card must have all required attributes
- Image URLs must point to valid static assets
- Links must be valid paths to documentation
- Order values must be unique and sequential
- Title and description must not exceed display limits

## Relationships

- Each card is independent but part of a collection displayed on the homepage
- Card order determines the visual arrangement on the grid