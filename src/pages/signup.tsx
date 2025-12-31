import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';

const SignupPage = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState('');
  const [hardwareExperience, setHardwareExperience] = useState('');
  const [programmingLanguages, setProgrammingLanguages] = useState('');
  const [goal, setGoal] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [clerkAvailable, setClerkAvailable] = useState(false);

  // Check if Clerk is available after component mounts (client-side only)
  useEffect(() => {
    // Check if Clerk is available on the client side
    if (typeof window !== 'undefined') {
      // Clerk availability check
      setClerkAvailable(!!(window as any).Clerk);
    }
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (!softwareLevel || !hardwareExperience || !goal) {
      setError('Please fill in all onboarding fields');
      return;
    }

    setLoading(true);
    setError('');

    // Try to use Clerk if available
    if (clerkAvailable) {
      try {
        const { useClerk } = await import('@clerk/clerk-react');
        // Use Clerk's sign-up flow
        const clerk = useClerk();
        if (clerk && clerk.openSignUp) {
          clerk.openSignUp();
        } else {
          setError('Authentication service is not available. Please try again later.');
          setLoading(false);
        }
      } catch (err) {
        setError('Authentication service is not available. Please try again later.');
        setLoading(false);
      }
    } else {
      setError('Authentication service is not configured. Please contact the administrator.');
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create your account for Physical AI course">
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
                <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600' }}>Create Your Account</h2>
                <p style={{ margin: '0.5rem 0 0 0', opacity: 0.9, fontSize: '0.9rem' }}>
                  Join the Physical AI & Humanoid Robotics course
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

                  <div className="margin-bottom--sm">
                    <label htmlFor="email" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Email Address</label>
                    <input
                      type="email"
                      id="email"
                      className="form-control"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      required
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    />
                  </div>

                  <div className="margin-bottom--sm">
                    <label htmlFor="password" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Password</label>
                    <input
                      type="password"
                      id="password"
                      className="form-control"
                      value={password}
                      onChange={(e) => setPassword(e.target.value)}
                      required
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    />
                  </div>

                  <div className="margin-bottom--sm">
                    <label htmlFor="confirmPassword" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Confirm Password</label>
                    <input
                      type="password"
                      id="confirmPassword"
                      className="form-control"
                      value={confirmPassword}
                      onChange={(e) => setConfirmPassword(e.target.value)}
                      required
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    />
                  </div>

                  <div className="margin-bottom--sm">
                    <label htmlFor="softwareLevel" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Software Level</label>
                    <select
                      id="softwareLevel"
                      className="form-control"
                      value={softwareLevel}
                      onChange={(e) => setSoftwareLevel(e.target.value)}
                      required
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        backgroundColor: 'white',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    >
                      <option value="" style={{ color: '#9ca3af' }}>Select your level</option>
                      <option value="beginner">Beginner</option>
                      <option value="intermediate">Intermediate</option>
                      <option value="advanced">Advanced</option>
                    </select>
                  </div>

                  <div className="margin-bottom--sm">
                    <label htmlFor="hardwareExperience" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Hardware / Robotics Experience</label>
                    <select
                      id="hardwareExperience"
                      className="form-control"
                      value={hardwareExperience}
                      onChange={(e) => setHardwareExperience(e.target.value)}
                      required
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        backgroundColor: 'white',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    >
                      <option value="" style={{ color: '#9ca3af' }}>Select your experience</option>
                      <option value="none">None</option>
                      <option value="basic">Basic</option>
                      <option value="advanced">Advanced</option>
                    </select>
                  </div>

                  <div className="margin-bottom--sm">
                    <label htmlFor="programmingLanguages" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Programming Languages (comma separated)</label>
                    <input
                      type="text"
                      id="programmingLanguages"
                      className="form-control"
                      value={programmingLanguages}
                      onChange={(e) => setProgrammingLanguages(e.target.value)}
                      placeholder="e.g., Python, C++, JavaScript"
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    />
                  </div>

                  <div className="margin-bottom--sm">
                    <label htmlFor="goal" className="form-label" style={{
                      display: 'block',
                      marginBottom: '0.5rem',
                      fontWeight: '500',
                      color: '#1f2937'
                    }}>Goal</label>
                    <select
                      id="goal"
                      className="form-control"
                      value={goal}
                      onChange={(e) => setGoal(e.target.value)}
                      required
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        border: '2px solid #e5e7eb',
                        borderRadius: '8px',
                        fontSize: '1rem',
                        backgroundColor: 'white',
                        transition: 'border-color 0.2s ease'
                      }}
                      onMouseOver={(e) => e.currentTarget.style.borderColor = '#2563eb'}
                      onMouseOut={(e) => e.currentTarget.style.borderColor = '#e5e7eb'}
                    >
                      <option value="" style={{ color: '#9ca3af' }}>Select your goal</option>
                      <option value="learning">Learning</option>
                      <option value="research">Research</option>
                      <option value="project">Project</option>
                      <option value="career">Career</option>
                    </select>
                  </div>

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
                        Creating Account...
                      </span>
                    ) : 'Sign Up'}
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
                  Already have an account?{' '}
                  <a
                    href="/signin"
                    style={{
                      color: '#2563eb',
                      textDecoration: 'none',
                      fontWeight: '500'
                    }}
                    onMouseOver={(e) => (e.target as HTMLElement).style.textDecoration = 'underline'}
                  >
                    Sign In
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

export default SignupPage;