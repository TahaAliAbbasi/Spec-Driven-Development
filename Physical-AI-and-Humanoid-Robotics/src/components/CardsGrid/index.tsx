import type {ReactNode} from 'react';
import clsx from 'clsx';
import Card from '@site/src/components/Card';
import styles from './styles.module.css';
import rawCardsData from '@site/src/data/homepage-cards.json';
import { validateAndSortCards } from '@site/src/utils/validateCards';

export default function CardsGrid(): ReactNode {
  const cardsData = validateAndSortCards(rawCardsData);

  return (
    <section className={clsx(styles.cardsGridSection, 'margin-vert--lg')}>
      <div className="container">
        <div className="row">
          {cardsData.map((card) => (
            <div key={card.id} className={clsx(styles.cardGridItem, 'margin-bottom--lg')}>
              <Card
                title={card.title}
                description={card.description}
                imageUrl={card.imageUrl}
                link={card.link}
                className={styles.cardItem}
              />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}