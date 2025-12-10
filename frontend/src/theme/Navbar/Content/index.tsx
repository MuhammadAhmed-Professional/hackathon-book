import React from 'react';
import Content from '@theme-original/Navbar/Content';
import { AuthProvider } from '../../../components/AuthProvider';
import ProfileButton from '../../../components/ProfileButton';

export default function ContentWrapper(props) {
  return (
    <AuthProvider>
      <>
        <Content {...props} />
        <div style={{ marginLeft: 'auto', display: 'flex', alignItems: 'center' }}>
          <ProfileButton />
        </div>
      </>
    </AuthProvider>
  );
}
