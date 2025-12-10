import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/SigninForm';

export default function SigninPage() {
  return (
    <Layout
      title="Sign In"
      description="Sign in to the Physical AI & Humanoid Robotics Textbook"
    >
      <SigninForm />
    </Layout>
  );
}
