/**
 * Custom DocItem Layout with SelectedTextHandler.
 * The floating chatbot is added globally via Root wrapper.
 */

import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import TextSelectionHandler from '../../../components/TextSelectionHandler';

export default function DocItemLayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      {/* TextSelectionHandler for text selection feature - opens FloatingChatbot */}
      <TextSelectionHandler />
    </>
  );
}
