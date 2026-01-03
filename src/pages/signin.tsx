import React from 'react';
import Layout from '@theme/Layout';
import { SignIn } from '@clerk/clerk-react';

const SigninPage = () => {
  return (
    <Layout title="Sign In" description="Sign in to your Physical AI course account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div style={{
              display: 'flex',
              justifyContent: 'center',
              alignItems: 'center',
              minHeight: '500px',
            }}>
              <SignIn
                appearance={{
                  elements: {
                    card: {
                      boxShadow: '0 10px 25px rgba(0,0,0,0.1)',
                      borderRadius: '12px',
                      border: 'none',
                      padding: '2rem',
                    },
                    headerTitle: {
                      fontSize: '1.5rem',
                      fontWeight: '600',
                      color: '#1f2937',
                    },
                    headerSubtitle: {
                      color: '#6b7280',
                      fontSize: '0.875rem',
                    },
                    socialButtonsBlockButton: {
                      border: '1px solid #e5e7eb',
                      borderRadius: '8px',
                      padding: '0.75rem',
                      backgroundColor: 'white',
                    },
                    formButtonPrimary: {
                      backgroundColor: '#2563eb',
                      padding: '0.75rem',
                      fontSize: '1rem',
                      fontWeight: '600',
                      borderRadius: '8px',
                      border: 'none',
                      cursor: 'pointer',
                      transition: 'background-color 0.2s ease',
                    },
                    formButtonPrimary__loading: {
                      backgroundColor: '#3b82f6',
                    },
                    footerActionText: {
                      color: '#6b7280',
                    },
                    footerActionLink: {
                      color: '#2563eb',
                      fontWeight: '500',
                    },
                    // Completely hide phone number related elements
                    formFieldInput__phoneNumber: {
                      display: 'none',
                    },
                    otpPhone: {
                      display: 'none',
                    },
                    alternativeMethodsBlockButton: {
                      display: 'none',
                    },
                    alternativeMethodsBlockButtonText: {
                      display: 'none',
                    },
                    alternativeMethods: {
                      display: 'none',
                    },
                    // Focus on email/password and OAuth
                    formFieldInput: {
                      marginBottom: '1rem',
                    },
                    formFieldInput__emailAddress: {
                      marginBottom: '1rem',
                    },
                    formFieldInput__password: {
                      marginBottom: '1rem',
                    },
                    socialButtons: {
                      display: 'flex',
                      gap: '0.5rem',
                      justifyContent: 'center',
                      marginBottom: '1rem',
                    },
                  }
                }}
                signUpUrl="/signup"
                routing="virtual"
              />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SigninPage;