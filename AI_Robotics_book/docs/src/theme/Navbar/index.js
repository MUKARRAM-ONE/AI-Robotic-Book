import React from 'react';
import Navbar from '@theme-original/Navbar';
import SearchBar from '../../components/SearchBar';
import AuthButton from '../../components/Auth/AuthButton';
import styles from './index.module.css';

export default function NavbarWrapper(props) {
  return (
    <>
      <div className={styles.navbarWithSearch}>
        <Navbar {...props} />
        <div className={styles.searchBarContainer}>
          <SearchBar navbar={true} />
          <AuthButton />
        </div>
      </div>
    </>
  );
}
