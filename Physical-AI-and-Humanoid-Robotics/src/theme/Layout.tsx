// Physical-AI-and-Humanoid-Robotics/src/theme/Layout.tsx
// Layout wrapper to add floating chatbot to every page

import React, { type ReactNode } from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatbot from '../components/FloatingChatbot';

interface LayoutProps {
  children: ReactNode;
  [key: string]: any;
}

export default function Layout(props: LayoutProps): ReactNode {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatbot />
    </>
  );
}