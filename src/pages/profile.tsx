import React from 'react';
import { useUser } from '@clerk/clerk-react';
import Layout from '@theme/Layout';

const ProfilePage = () => {
  const { user, isLoaded, isSignedIn } = useUser();

  if (!isLoaded) {
    return (
      <Layout title="Loading" description="Loading user profile...">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="text--center padding--vert--lg">
                <h1>Loading...</h1>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isSignedIn) {
    return (
      <Layout title="Profile" description="User profile page">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="card">
                <div className="card__header" style={{
                  backgroundColor: '#2563eb',
                  color: 'white',
                  padding: '1.5rem',
                  textAlign: 'center'
                }}>
                  <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600' }}>Access Denied</h2>
                </div>
                <div className="card__body" style={{ padding: '1.5rem', textAlign: 'center' }}>
                  <p>Please sign in to view your profile.</p>
                  <a
                    href="/my-physical-ai-book/signin"
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
                    Sign In
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
    <Layout title="Profile" description="User profile page">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="card">
              <div className="card__header" style={{
                backgroundColor: '#2563eb',
                color: 'white',
                padding: '1.5rem',
                textAlign: 'center'
              }}>
                <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600' }}>Your Profile</h2>
              </div>
              <div className="card__body" style={{ padding: '2rem' }}>
                <div className="margin-bottom--lg">
                  <h3 style={{ color: '#1f2937', marginBottom: '1rem' }}>Personal Information</h3>
                  <div className="row">
                    <div className="col col--6">
                      <p><strong>Name:</strong> {user.fullName || user.username || 'N/A'}</p>
                      <p><strong>Email:</strong> {user.primaryEmailAddress?.emailAddress || 'N/A'}</p>
                      <p><strong>Member Since:</strong> {user.createdAt ? new Date(user.createdAt).toLocaleDateString() : 'N/A'}</p>
                    </div>
                    <div className="col col--6">
                      <p><strong>Software Level:</strong> {user.unsafeMetadata?.softwareLevel || 'N/A'}</p>
                      <p><strong>Hardware Experience:</strong> {user.unsafeMetadata?.hardwareExperience || 'N/A'}</p>
                      <p><strong>Goal:</strong> {user.unsafeMetadata?.goal || 'N/A'}</p>
                    </div>
                  </div>
                </div>

                <div className="margin-bottom--lg">
                  <h3 style={{ color: '#1f2937', marginBottom: '1rem' }}>Programming Languages</h3>
                  <p>{user.unsafeMetadata?.programmingLanguages || 'N/A'}</p>
                </div>

                <div className="margin-bottom--lg">
                  <h3 style={{ color: '#1f2937', marginBottom: '1rem' }}>Security</h3>
                  <p><strong>Two-Factor Authentication:</strong> {user.twoFactorEnabled ? 'Enabled' : 'Disabled'}</p>
                </div>

                <div className="button-group button-group--block">
                  <a
                    href="/my-physical-ai-book/"
                    className="button button--secondary"
                    style={{
                      marginRight: '1rem',
                      padding: '0.5rem 1rem',
                      borderRadius: '8px',
                      fontSize: '1rem',
                      textDecoration: 'none'
                    }}
                  >
                    Back to Course
                  </a>
                  <a
                    href="#"
                    onClick={(e) => {
                      e.preventDefault();
                      (window as any).Clerk?.signOut().then(() => {
                        window.location.href = '/my-physical-ai-book/';
                      });
                    }}
                    className="button button--danger"
                    style={{
                      padding: '0.5rem 1rem',
                      backgroundColor: '#dc2626',
                      color: 'white',
                      border: 'none',
                      borderRadius: '8px',
                      fontSize: '1rem',
                      cursor: 'pointer',
                      textDecoration: 'none'
                    }}
                  >
                    Sign Out
                  </a>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ProfilePage;