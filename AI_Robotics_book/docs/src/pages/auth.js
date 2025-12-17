import React from 'react';
import Layout from '@theme/Layout';
import { useLocation } from '@docusaurus/router';
import Login from '../components/Auth/Login';
import Signup from '../components/Auth/Signup';

function AuthPage() {
  const location = useLocation();
  const params = new URLSearchParams(location.search);
  const action = params.get('action');

  return (
    <Layout title="Auth">
      {action === 'signup' ? <Signup /> : <Login />}
    </Layout>
  );
}

export default AuthPage;
