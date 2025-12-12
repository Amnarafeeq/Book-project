import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

function Footer() {
  return (
    <footer className={clsx('footer', styles.footer)}>
      <div className="container">
        <div className="row">
          <div className="col col--3">
            <h4 className={styles.footerTitle}>Academy</h4>
            <ul className="footer__items">
              <li className="footer__item">
                <Link to="/docs/modules" className={styles.footerLink}>Modules</Link>
              </li>
              <li className="footer__item">
                <Link to="/docs/overview" className={styles.footerLink}>Course Overview</Link>
              </li>
              <li className="footer__item">
                <Link to="/docs/capstone" className={styles.footerLink}>Capstone Project</Link>
              </li>
            </ul>
          </div>

          <div className="col col--3">
            <h4 className={styles.footerTitle}>Resources</h4>
            <ul className="footer__items">
              <li className="footer__item">
                <Link to="/docs/glossary" className={styles.footerLink}>Glossary</Link>
              </li>
              <li className="footer__item">
                <Link to="/docs/hardware" className={styles.footerLink}>Hardware Requirements</Link>
              </li>
              <li className="footer__item">
                <Link href="https://github.com/your-organization/physical-ai-textbook" className={styles.footerLink}>GitHub Repository</Link>
              </li>
            </ul>
          </div>

          <div className="col col--3">
            <h4 className={styles.footerTitle}>Community</h4>
            <ul className="footer__items">
              <li className="footer__item">
                <Link href="https://discordapp.com/invite/your-discord" className={styles.footerLink}>Discord</Link>
              </li>
              <li className="footer__item">
                <Link href="https://twitter.com/your-twitter" className={styles.footerLink}>Twitter</Link>
              </li>
              <li className="footer__item">
                <Link href="https://forum.your-robotics-academy.com" className={styles.footerLink}>Forum</Link>
              </li>
            </ul>
          </div>

          <div className="col col--3">
            <h4 className={styles.footerTitle}>Legal</h4>
            <ul className="footer__items">
              <li className="footer__item">
                <Link to="/privacy" className={styles.footerLink}>Privacy Policy</Link>
              </li>
              <li className="footer__item">
                <Link to="/terms" className={styles.footerLink}>Terms of Service</Link>
              </li>
              <li className="footer__item">
                <Link to="/license" className={styles.footerLink}>License</Link>
              </li>
            </ul>
          </div>
        </div>

        <div className="footer__bottom text--center">
          <div className="footer__copyright">
            Copyright © {new Date().getFullYear()} Physical AI & Humanoid Robotics Academy. All rights reserved.
          </div>
        </div>
      </div>
    </footer>
  );
}

export default Footer;