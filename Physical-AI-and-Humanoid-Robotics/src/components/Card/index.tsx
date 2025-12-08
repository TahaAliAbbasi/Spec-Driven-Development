import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './styles.module.css';

interface CardProps {
  title: string;
  description: string;
  imageUrl: string;
  link: string;
  className?: string;
}

export default function Card({title, description, imageUrl, link, className}: CardProps): ReactNode {
  return (
    <div className={clsx('card', styles.card, className)} role="article" aria-label={title}>
      <div className={styles.cardImage}>
        <img
          src={imageUrl}
          alt={`${title} - ${description}`}
          loading="lazy"
          onError={(e) => {
            const target = e.target as HTMLImageElement;
            target.onerror = null; // Prevent infinite loop if fallback also fails
            target.src = '/img/docusaurus.png'; // Fallback image
          }}
        />
      </div>
      <div className={styles.cardBody}>
        <h3 className={styles.cardTitle}>{title}</h3>
        <p className={styles.cardDescription}>{description}</p>
        <div className={styles.cardFooter}>
          <Link
            className={clsx('button button--primary', styles.cardButton)}
            to={link}
            aria-label={`Read more about ${title}`}
          >
            Read More
          </Link>
        </div>
      </div>
    </div>
    
  );
}