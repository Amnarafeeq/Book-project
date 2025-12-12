import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

function CalloutBox({type, title, children}) {
  const calloutClasses = clsx(
    'alert',
    styles.calloutBox,
    {
      'alert--primary': type === 'note',
      'alert--info': type === 'info',
      'alert--warning': type === 'caution',
      'alert--success': type === 'tip',
    }
  );

  const icon = {
    note: '📝',
    info: 'ℹ️',
    caution: '⚠️',
    tip: '💡'
  }[type] || '📌';

  return (
    <div className={calloutClasses}>
      <div className={styles.calloutHeader}>
        <span className={styles.calloutIcon}>{icon}</span>
        <span className={styles.calloutTitle}>{title || type.charAt(0).toUpperCase() + type.slice(1)}</span>
      </div>
      <div className={styles.calloutContent}>
        {children}
      </div>
    </div>
  );
}

export default CalloutBox;