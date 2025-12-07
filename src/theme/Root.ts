import React from 'react';
import type { Props } from '@theme/Root';
import Chatbot from '../components/Chatbot';

export default function Root({ children }: Props) {
  return React.createElement(React.Fragment, null, children, React.createElement(Chatbot, null));
}
