import React, { useState } from 'react';
import { useAuth } from '../components/ClerkAuthProvider';
import { useUser, useClerk } from '@clerk/clerk-react';
import Layout from '@theme/Layout';
import clsx from 'clsx';

const SigninPage = () => {
  const { user } = useUser();
  const { openSignIn } = useClerk();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    setLoading(true);
    setError('');

    try {
      // Open Clerk's sign-in flow
      openSignIn();
    } catch (err) {
      setError('An error occurred during sign in. Please try again.');
      console.error(err);
      setLoading(false);
    }
  };

  // If user is already signed in, redirect them or show a message
  if (user) {
    return (
      <Layout title="Sign In" description="Sign in to your Physical AI course account">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="card" style={{
                boxShadow: '0 10px 25px rgba(0,0,0,0.1)',
                borderRadius: '12px',
                overflow: 'hidden',
                border: 'none'
              }}>
                <div className="card__header" style={{
                  backgroundColor: '#2563eb',
                  color: 'white',
                  padding: '1.5rem',
                  textAlign: 'center'
                }}>
                  <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600' }}>Already Signed In</h2>
                </div>
                <div className="card__body" style={{ padding: '1.5rem', textAlign: 'center' }}>
                  <p>You are already signed in as {user.primaryEmailAddress?.emailAddress}</p>
                  <a
                    href="/my-physical-ai-book/profile"
                    className="button button--primary"
                    style={{
                      padding: '0.5rem 1rem',
                      backgroundColor: '#2563eb',
                      color: 'white',
                      border: 'none',
                      borderRadius: '8px',
                      fontSize: '1rem',
                      fontWeight: '600',
                      cursor: 'pointer',
                      textDecoration: 'none'
                    }}
                  >
                    View Profile
                  </a>
                </div>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign In" description="Sign in to your Physical AI course account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card" style={{
              boxShadow: '0 10px 25px rgba(0,0,0,0.1)',
              borderRadius: '12px',
              overflow: 'hidden',
              border: 'none'
            }}>
              <div className="card__header" style={{
                backgroundColor: '#2563eb',
                color: 'white',
                padding: '1.5rem',
                textAlign: 'center'
              }}>
                <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600' }}>Welcome Back</h2>
                <p style={{ margin: '0.5rem 0 0 0', opacity: 0.9, fontSize: '0.9rem' }}>
                  Sign in to your Physical AI & Humanoid Robotics account
                </p>
              </div>
              <div className="card__body" style={{ padding: '1.5rem' }}>
                <form onSubmit={handleSubmit}>
                  {error && (
                    <div className="alert alert--danger margin-bottom--md" style={{
                      backgroundColor: '#fee2e2',
                      color: '#dc2626',
                      border: '1px solid #fecaca',
                      borderRadius: '8px',
                      padding: '0.75rem'
                    }}>
                      {error}
                    </div>
                  )}

                  <p style={{ textAlign: 'center', color: '#6b7280', marginBottom: '1rem' }}>
                    Sign in using your preferred method
                  </p>

                  <button
                    type="submit"
                    className={clsx('button button--primary button--block', {
                      'button--loading': loading,
                    })}
                    disabled={loading}
                    style={{
                      width: '100%',
                      padding: '0.75rem',
                      backgroundColor: '#2563eb',
                      color: 'white',
                      border: 'none',
                      borderRadius: '8px',
                      fontSize: '1rem',
                      fontWeight: '600',
                      cursor: 'pointer',
                      transition: 'background-color 0.2s ease, transform 0.1s ease',
                      marginTop: '1rem'
                    }}
                    onMouseOver={(e) => {
                      (e.target as HTMLElement).style.backgroundColor = '#1d4ed8';
                      (e.target as HTMLElement).style.transform = 'translateY(-2px)';
                    }}
                    onMouseOut={(e) => {
                      (e.target as HTMLElement).style.backgroundColor = '#2563eb';
                      (e.target as HTMLElement).style.transform = 'translateY(0)';
                    }}
                  >
                    {loading ? (
                      <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                        <span className="button__loader" style={{
                          width: '16px',
                          height: '16px',
                          border: '2px solid transparent',
                          borderTop: '2px solid currentColor',
                          borderRadius: '50%',
                          animation: 'spin 1s linear infinite',
                          marginRight: '0.5rem'
                        }}></span>
                        Signing In...
                      </span>
                    ) : 'Continue with Email/Password'}
                  </button>
                </form>
              </div>
              <div className="card__footer" style={{
                padding: '1rem',
                backgroundColor: '#f9fafb',
                borderTop: '1px solid #e5e7eb',
                textAlign: 'center'
              }}>
                <p style={{ margin: 0, color: '#6b7280' }}>
                  Don't have an account?{' '}
                  <a
                    href="/my-physical-ai-book/signup"
                    style={{
                      color: '#2563eb',
                      textDecoration: 'none',
                      fontWeight: '500'
                    }}
                    onMouseOver={(e) => (e.target as HTMLElement).style.textDecoration = 'underline'}
                  >
                    Sign Up
                  </a>
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>

      <style jsx>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </Layout>
  );
};

export default SigninPage;