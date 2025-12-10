import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/SignupForm';

export default function SignupPage() {
  return (
    <Layout
      title="Sign Up"
      description="Create your account for the Physical AI & Humanoid Robotics Textbook"
    >
      <SignupForm />
    </Layout>
  );
}
