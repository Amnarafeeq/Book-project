import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import {NavbarLogo} from '@docusaurus/theme-common';
import {useLocation} from '@docusaurus/router';
import styles from './styles.module.css';

function Navbar() {
  const {siteConfig} = useDocusaurusContext();
  const location = useLocation();

  return (
    <nav
      className={clsx('navbar', 'navbar--primary', 'navbar--fixed-top', styles.navbar)}
    >
      <div className="navbar__inner">
        <div className="navbar__items">
          <NavbarLogo />
          <Link className="navbar__title" to={useBaseUrl('/')}>
            {siteConfig.title}
          </Link>
        </div>
        <div className="navbar__items navbar__items--right">
          <Link
            className={clsx(
              'navbar__item navbar__link',
              location.pathname.includes('/docs/modules') && 'navbar__link--active'
            )}
            to={useBaseUrl('/docs/modules')}
          >
            Modules
          </Link>
          <Link
            className={clsx(
              'navbar__item navbar__link',
              location.pathname.includes('/docs/overview') && 'navbar__link--active'
            )}
            to={useBaseUrl('/docs/overview')}
          >
            Course Overview
          </Link>
          <Link
            className={clsx(
              'navbar__item navbar__link',
              location.pathname.includes('/docs/hardware') && 'navbar__link--active'
            )}
            to={useBaseUrl('/docs/hardware')}
          >
            Hardware
          </Link>
          <Link
            className={clsx(
              'navbar__item navbar__link',
              location.pathname.includes('/docs/capstone') && 'navbar__link--active'
            )}
            to={useBaseUrl('/docs/capstone')}
          >
            Capstone
          </Link>
          <Link
            className={clsx(
              'navbar__item navbar__link',
              location.pathname.includes('/docs/glossary') && 'navbar__link--active'
            )}
            to={useBaseUrl('/docs/glossary')}
          >
            Glossary
          </Link>
          <Link
            className={clsx(
              'navbar__item navbar__link',
              location.pathname.includes('/docs/resources') && 'navbar__link--active'
            )}
            to={useBaseUrl('/docs/resources')}
          >
            Resources
          </Link>
        </div>
      </div>
    </nav>
  );
}

export default Navbar;