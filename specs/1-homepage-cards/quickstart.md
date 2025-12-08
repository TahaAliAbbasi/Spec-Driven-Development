# Quickstart Guide: Homepage Cards Implementation

## Overview
This guide provides a quick reference for implementing the homepage cards feature for the Docusaurus website.


## Prerequisites
- Node.js and npm installed
- Docusaurus project set up
- Access to image assets for cards

## Files to Create/Modify

### 1. Card Data Configuration
Create `src/data/homepage-cards.json` with card information:
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

### 2. Card Component
Create `src/components/Card/index.js`:
```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './styles.module.css';

const Card = ({title, description, imageUrl, link, className}) => {
  return (
    <div className={clsx('card', className)}>
      <div className="card__header">
        {imageUrl && (
          <img src={imageUrl} alt={title} className="card__image" />
        )}
        <h3 className="card__title">{title}</h3>
      </div>
      <div className="card__body">
        <p>{description}</p>
      </div>
      <div className="card__footer">
        <Link className="button button--primary" to={link}>
          Read More
        </Link>
      </div>
    </div>
  );
};

export default Card;
```

### 3. Card Styles
Create `src/components/Card/styles.module.css`:
```css
.card {
  display: flex;
  flex-direction: column;
  height: 100%;
  border: 1px solid var(--ifm-color-emphasis-200);
  border-radius: var(--ifm-global-radius);
  box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.07), 0 1px 3px 0 rgba(0, 0, 0, 0.05);
  transition: transform 0.25s ease, box-shadow 0.25s ease;
}

.card:hover {
  transform: translateY(-5px);
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
}

.card__header {
  padding: var(--ifm-card-vertical-spacing) var(--ifm-card-horizontal-spacing);
  border-bottom: 1px solid var(--ifm-color-emphasis-200);
}

.card__image {
  width: 100%;
  max-height: 150px;
  object-fit: cover;
  border-radius: var(--ifm-global-radius) var(--ifm-global-radius) 0 0;
  margin-bottom: calc(var(--ifm-global-spacing) / 2);
}

.card__title {
  margin-bottom: 0;
  font-size: 1.25rem;
  font-weight: var(--ifm-font-weight-bold);
}

.card__body {
  padding: var(--ifm-card-vertical-spacing) var(--ifm-card-horizontal-spacing);
  flex: 1;
}

.card__footer {
  padding: var(--ifm-card-vertical-spacing) var(--ifm-card-horizontal-spacing);
  border-top: 1px solid var(--ifm-color-emphasis-200);
  text-align: right;
}

@media (min-width: 997px) {
  .card__image {
    max-height: 100px;
  }
}
```

### 4. Homepage Cards Section
Add to the homepage (typically `src/pages/index.js` or in an MDX file):
```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Card from '@site/src/components/Card';
import cardData from '@site/src/data/homepage-cards.json';
import styles from './index.module.css';

function HomepageCards() {
  return (
    <section className={clsx('container', styles.homepageCards)}>
      <div className="row">
        {cardData.map((card, index) => (
          <div key={card.id} className="col col--4 margin-bottom--lg">
            <Card
              title={card.title}
              description={card.description}
              imageUrl={card.imageUrl}
              link={card.link}
            />
          </div>
        ))}
      </div>
    </section>
  );
}

export default HomepageCards;
```

## Development Commands
- `npm start` - Start the development server
- `npm run build` - Build the static website
- `npm run serve` - Serve the built website locally

## Testing
1. Verify all 5 cards appear on the homepage
2. Check that images load correctly
3. Test that "Read More" buttons navigate to correct documentation
4. Validate responsive behavior on different screen sizes
5. Ensure accessibility features work properly