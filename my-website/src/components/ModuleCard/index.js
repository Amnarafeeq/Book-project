import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

function ModuleCard({title, description, icon, link, skills, tools}) {
  return (
    <div className={clsx('card', styles.moduleCard)}>
      <div className="card__header">
        <h3 className={styles.cardTitle}>
          {icon && <span className={styles.cardIcon}>{icon}</span>}
          {title}
        </h3>
      </div>
      <div className="card__body">
        <p className={styles.cardDescription}>{description}</p>

        {skills && (
          <div className={styles.cardSection}>
            <h4>Skills You Will Learn:</h4>
            <ul className={styles.skillsList}>
              {skills.map((skill, index) => (
                <li key={index} className={styles.skillItem}>{skill}</li>
              ))}
            </ul>
          </div>
        )}

        {tools && (
          <div className={styles.cardSection}>
            <h4>Tools You Will Use:</h4>
            <ul className={styles.toolsList}>
              {tools.map((tool, index) => (
                <li key={index} className={styles.toolItem}>{tool}</li>
              ))}
            </ul>
          </div>
        )}
      </div>

      {link && (
        <div className="card__footer">
          <Link className={clsx('button', 'button--primary', styles.cardButton)} to={link}>
            Start Module
          </Link>
        </div>
      )}
    </div>
  );
}

export default ModuleCard;