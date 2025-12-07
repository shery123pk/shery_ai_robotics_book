/**
 * AuthPanel component - User authentication (login/signup)
 */

import React, { useState } from 'react';

interface User {
  id: string;
  email: string;
  full_name: string;
  role: string;
}

export default function AuthPanel(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [mode, setMode] = useState<'login' | 'signup'>('login');
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [fullName, setFullName] = useState('');
  const [background, setBackground] = useState('');

  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        throw new Error('Login failed');
      }

      const data = await response.json();
      setUser(data.user);
      localStorage.setItem('auth_token', data.access_token);
      setIsOpen(false);
      setEmail('');
      setPassword('');
    } catch (err) {
      setError('Invalid email or password');
    } finally {
      setLoading(false);
    }
  };

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email,
          password,
          full_name: fullName,
          background,
        }),
      });

      if (!response.ok) {
        throw new Error('Signup failed');
      }

      const data = await response.json();
      setUser(data.user);
      localStorage.setItem('auth_token', data.access_token);
      setIsOpen(false);
      setEmail('');
      setPassword('');
      setFullName('');
      setBackground('');
    } catch (err) {
      setError('Signup failed. Email may already be registered.');
    } finally {
      setLoading(false);
    }
  };

  const handleLogout = () => {
    setUser(null);
    localStorage.removeItem('auth_token');
  };

  return (
    <>
      {/* Auth Button */}
      <button
        onClick={() => (user ? handleLogout() : setIsOpen(true))}
        style={{
          position: 'fixed',
          top: '70px',
          right: '20px',
          padding: '8px 16px',
          background: user ? 'var(--ifm-color-success)' : 'var(--ifm-color-primary)',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer',
          zIndex: 999,
          fontSize: '14px',
        }}
      >
        {user ? `👤 ${user.full_name}` : '🔐 Login'}
      </button>

      {/* Auth Modal */}
      {isOpen && !user && (
        <div style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          background: 'rgba(0,0,0,0.5)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: 1001,
        }}>
          <div style={{
            background: 'var(--ifm-background-color)',
            padding: '32px',
            borderRadius: '8px',
            maxWidth: '400px',
            width: '90%',
            boxShadow: '0 4px 20px rgba(0,0,0,0.3)',
          }}>
            <h2 style={{ marginTop: 0 }}>
              {mode === 'login' ? 'Login' : 'Sign Up'}
            </h2>

            <form onSubmit={mode === 'login' ? handleLogin : handleSignup}>
              {mode === 'signup' && (
                <div style={{ marginBottom: '16px' }}>
                  <label style={{ display: 'block', marginBottom: '4px' }}>Full Name</label>
                  <input
                    type="text"
                    value={fullName}
                    onChange={(e) => setFullName(e.target.value)}
                    required
                    style={{
                      width: '100%',
                      padding: '8px',
                      borderRadius: '4px',
                      border: '1px solid var(--ifm-color-emphasis-300)',
                    }}
                  />
                </div>
              )}

              <div style={{ marginBottom: '16px' }}>
                <label style={{ display: 'block', marginBottom: '4px' }}>Email</label>
                <input
                  type="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  style={{
                    width: '100%',
                    padding: '8px',
                    borderRadius: '4px',
                    border: '1px solid var(--ifm-color-emphasis-300)',
                  }}
                />
              </div>

              <div style={{ marginBottom: '16px' }}>
                <label style={{ display: 'block', marginBottom: '4px' }}>Password</label>
                <input
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  minLength={8}
                  style={{
                    width: '100%',
                    padding: '8px',
                    borderRadius: '4px',
                    border: '1px solid var(--ifm-color-emphasis-300)',
                  }}
                />
              </div>

              {mode === 'signup' && (
                <div style={{ marginBottom: '16px' }}>
                  <label style={{ display: 'block', marginBottom: '4px' }}>
                    Background (optional)
                  </label>
                  <textarea
                    value={background}
                    onChange={(e) => setBackground(e.target.value)}
                    placeholder="e.g., Computer Science student, Robotics engineer..."
                    rows={2}
                    style={{
                      width: '100%',
                      padding: '8px',
                      borderRadius: '4px',
                      border: '1px solid var(--ifm-color-emphasis-300)',
                    }}
                  />
                </div>
              )}

              {error && (
                <div style={{
                  color: 'var(--ifm-color-danger)',
                  marginBottom: '16px',
                  fontSize: '14px',
                }}>
                  {error}
                </div>
              )}

              <div style={{ display: 'flex', gap: '8px' }}>
                <button
                  type="submit"
                  disabled={loading}
                  style={{
                    flex: 1,
                    padding: '10px',
                    background: 'var(--ifm-color-primary)',
                    color: 'white',
                    border: 'none',
                    borderRadius: '4px',
                    cursor: loading ? 'not-allowed' : 'pointer',
                    opacity: loading ? 0.6 : 1,
                  }}
                >
                  {loading ? 'Please wait...' : (mode === 'login' ? 'Login' : 'Sign Up')}
                </button>
                <button
                  type="button"
                  onClick={() => setIsOpen(false)}
                  style={{
                    padding: '10px 20px',
                    background: 'transparent',
                    color: 'var(--ifm-color-emphasis-600)',
                    border: '1px solid var(--ifm-color-emphasis-300)',
                    borderRadius: '4px',
                    cursor: 'pointer',
                  }}
                >
                  Cancel
                </button>
              </div>
            </form>

            <div style={{ marginTop: '16px', textAlign: 'center', fontSize: '14px' }}>
              {mode === 'login' ? (
                <>
                  Don't have an account?{' '}
                  <button
                    onClick={() => setMode('signup')}
                    style={{
                      background: 'none',
                      border: 'none',
                      color: 'var(--ifm-color-primary)',
                      cursor: 'pointer',
                      textDecoration: 'underline',
                    }}
                  >
                    Sign up
                  </button>
                </>
              ) : (
                <>
                  Already have an account?{' '}
                  <button
                    onClick={() => setMode('login')}
                    style={{
                      background: 'none',
                      border: 'none',
                      color: 'var(--ifm-color-primary)',
                      cursor: 'pointer',
                      textDecoration: 'underline',
                    }}
                  >
                    Login
                  </button>
                </>
              )}
            </div>
          </div>
        </div>
      )}
    </>
  );
}
