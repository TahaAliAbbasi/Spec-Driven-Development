interface CardData {
  id: string;
  title: string;
  description: string;
  imageUrl: string;
  link: string;
  order: number;
}

export function validateCardData(cards: unknown): cards is CardData[] {
  if (!Array.isArray(cards)) {
    console.error('Card data is not an array');
    return false;
  }

  for (const card of cards) {
    if (!card || typeof card !== 'object') {
      console.error('Card item is not an object:', card);
      return false;
    }

    if (!card.hasOwnProperty('id') || typeof card.id !== 'string' || !card.id.trim()) {
      console.error('Card missing or invalid id:', card);
      return false;
    }

    if (!card.hasOwnProperty('title') || typeof card.title !== 'string' || !card.title.trim()) {
      console.error('Card missing or invalid title:', card);
      return false;
    }

    if (!card.hasOwnProperty('description') || typeof card.description !== 'string' || !card.description.trim()) {
      console.error('Card missing or invalid description:', card);
      return false;
    }

    if (!card.hasOwnProperty('imageUrl') || typeof card.imageUrl !== 'string' || !card.imageUrl.trim()) {
      console.error('Card missing or invalid imageUrl:', card);
      return false;
    }

    if (!card.hasOwnProperty('link') || typeof card.link !== 'string' || !card.link.trim()) {
      console.error('Card missing or invalid link:', card);
      return false;
    }

    // Validate link format - should start with /docs/ or / for internal links
    if (!card.link.startsWith('/docs/') && !card.link.startsWith('/') && !card.link.startsWith('http')) {
      console.error('Card link is not a valid internal or external URL:', card.link);
      return false;
    }

    if (!card.hasOwnProperty('order') || typeof card.order !== 'number' || card.order < 0) {
      console.error('Card missing or invalid order:', card);
      return false;
    }
  }

  return true;
}

export function validateAndSortCards(cards: unknown): CardData[] {
  if (!validateCardData(cards)) {
    console.error('Card data validation failed, using empty array as fallback');
    return [];
  }

  // Sort cards by order property
  return [...cards].sort((a, b) => a.order - b.order);
}