import React from 'react';
import clsx from 'clsx';
import Highlight, { defaultProps } from 'prism-react-renderer';
import styles from './styles.module.css';

function CodeBlock({children, className, language}) {
  const lang = className ? className.replace(/language-/, '') : language || 'javascript';

  return (
    <div className={clsx('code-block', styles.codeBlock)}>
      <Highlight
        {...defaultProps}
        code={children.trim()}
        language={lang}
      >
        {({ className, style, tokens, getLineProps, getTokenProps }) => (
          <pre className={className} style={{ ...style, borderRadius: '0.5rem', boxShadow: '0 0 10px rgba(167, 139, 250, 0.5)' }}>
            <code>
              {tokens.map((line, i) => (
                <div key={i} {...getLineProps({ line })}>
                  {line.map((token, key) => (
                    <span key={key} {...getTokenProps({ token })} />
                  ))}
                </div>
              ))}
            </code>
          </pre>
        )}
      </Highlight>
    </div>
  );
}

export default CodeBlock;