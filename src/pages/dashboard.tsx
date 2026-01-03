import React from 'react';
import Layout from '@theme/Layout';
import { useUser } from '@clerk/clerk-react';

const DashboardPage = () => {
  const { user, isSignedIn, isLoaded } = useUser();

  if (!isLoaded) {
    return (
      <Layout title="Dashboard" description="Loading dashboard...">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="text--center padding--vert--lg">
                <h1>Loading Dashboard...</h1>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isSignedIn) {
    return (
      <Layout title="Dashboard" description="Please sign in to access the dashboard">
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
                  <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600' }}>Access Denied</h2>
                </div>
                <div className="card__body" style={{ padding: '2rem', textAlign: 'center' }}>
                  <p>You need to sign in to access the dashboard.</p>
                  <div className="button-group margin-top--md">
                    <a
                      href="/signin"
                      className="button button--primary button--lg"
                      style={{
                        textDecoration: 'none',
                        marginRight: '1rem'
                      }}
                    >
                      Sign In
                    </a>
                    <a
                      href="/signup"
                      className="button button--secondary button--lg"
                      style={{
                        textDecoration: 'none'
                      }}
                    >
                      Create Account
                    </a>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="Your Personal Dashboard">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <div className="text--center margin-bottom--lg">
              <h1>Welcome to Your Dashboard</h1>
              <p style={{ fontSize: '1.2rem', color: '#6b7280' }}>
                Manage your account and access your resources
              </p>
            </div>

            <div className="row">
              <div className="col col--4">
                <div className="card">
                  <div className="card__header">
                    <h3>Profile</h3>
                  </div>
                  <div className="card__body">
                    <p><strong>Name:</strong> {user.fullName || user.username || 'N/A'}</p>
                    <p><strong>Email:</strong> {user.primaryEmailAddress?.emailAddress || 'N/A'}</p>
                    <p><strong>Member Since:</strong> {user.createdAt ? new Date(user.createdAt).toLocaleDateString() : 'N/A'}</p>
                  </div>
                  <div className="card__footer">
                    <a href="#" className="button button--secondary button--block">
                      View Profile
                    </a>
                  </div>
                </div>
              </div>

              <div className="col col--4">
                <div className="card">
                  <div className="card__header">
                    <h3>Learning Progress</h3>
                  </div>
                  <div className="card__body">
                    <p>Track your progress in Physical AI & Humanoid Robotics</p>
                    <div className="progress-bar" style={{
                      height: '10px',
                      backgroundColor: '#e5e7eb',
                      borderRadius: '5px',
                      margin: '1rem 0',
                      overflow: 'hidden'
                    }}>
                      <div style={{
                        height: '100%',
                        width: '25%', // Example progress
                        backgroundColor: '#2563eb',
                        transition: 'width 0.3s ease'
                      }}></div>
                    </div>
                    <p>25% Complete</p>
                  </div>
                  <div className="card__footer">
                    <a href="/docs/intro" className="button button--secondary button--block">
                      Continue Learning
                    </a>
                  </div>
                </div>
              </div>

              <div className="col col--4">
                <div className="card">
                  <div className="card__header">
                    <h3>Quick Actions</h3>
                  </div>
                  <div className="card__body">
                    <p>Access your most important resources</p>
                  </div>
                  <div className="card__footer">
                    <div className="button-group button-group--vertical">
                      <a href="/docs/intro" className="button button--outline button--block margin-bottom--sm">
                        Course Materials
                      </a>
                      <a href="#" className="button button--outline button--block margin-bottom--sm">
                        Ask Questions
                      </a>
                      <a href="#" className="button button--outline button--block">
                        Settings
                      </a>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            <div className="margin-vert--lg">
              <div className="text--center">
                <h2>Recent Activity</h2>
                <p>No recent activity yet. Start exploring the course materials!</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default DashboardPage;