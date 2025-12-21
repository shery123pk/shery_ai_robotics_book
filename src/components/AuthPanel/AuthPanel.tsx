/**
 * AuthPanel component - User authentication (login/signup)
 */

import React, { useState } from 'react';
import { getApiUrl } from '../../utils/apiConfig';

interface User {
  id: string;
  email: string;
  full_name: string;
  role: string;
}

export default function AuthPanel(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [showUserMenu, setShowUserMenu] = useState(false);
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
      const response = await fetch(getApiUrl('/api/auth/login'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Login failed');
      }

      const data = await response.json();
      // Backend returns nested user object in TokenResponse
      setUser({
        id: data.user.id,
        email: data.user.email,
        full_name: data.user.full_name,
        role: data.user.role,
      });
      localStorage.setItem('auth_token', data.access_token);
      setIsOpen(false);
      setEmail('');
      setPassword('');
    } catch (err: any) {
      setError(err.message || 'Invalid email or password');
    } finally {
      setLoading(false);
    }
  };

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch(getApiUrl('/api/auth/signup'), {
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
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Signup failed');
      }

      const data = await response.json();
      // Backend returns TokenResponse with user object and access_token
      setUser({
        id: data.user.id,
        email: data.user.email,
        full_name: data.user.full_name,
        role: data.user.role,
      });
      localStorage.setItem('auth_token', data.access_token);
      setIsOpen(false);
      setEmail('');
      setPassword('');
      setFullName('');
      setBackground('');
    } catch (err: any) {
      setError(err.message || 'Signup failed. Email may already be registered.');
    } finally {
      setLoading(false);
    }
  };

  const handleLogout = () => {
    setUser(null);
    setShowUserMenu(false);
    localStorage.removeItem('auth_token');
  };

  return (
    <>
      {/* Auth Button */}
      {user ? (
        <div style={{ position: 'fixed', top: '70px', right: '20px', zIndex: 999 }}>
          <button
            onClick={() => setShowUserMenu(!showUserMenu)}
            style={{
              padding: '8px 16px',
              background: 'var(--ifm-color-success)',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '14px',
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
            }}
          >
            👤 {user.full_name} ▼
          </button>

          {/* User Dropdown Menu */}
          {showUserMenu && (
            <div style={{
              position: 'absolute',
              top: '100%',
              right: 0,
              marginTop: '4px',
              background: 'var(--ifm-background-color)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '4px',
              boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
              minWidth: '180px',
              overflow: 'hidden',
            }}>
              <div style={{
                padding: '12px 16px',
                borderBottom: '1px solid var(--ifm-color-emphasis-200)',
                fontSize: '13px',
              }}>
                <div style={{ fontWeight: 'bold', marginBottom: '4px' }}>{user.full_name}</div>
                <div style={{ color: 'var(--ifm-color-emphasis-600)', fontSize: '12px' }}>{user.email}</div>
              </div>
              <button
                onClick={handleLogout}
                style={{
                  width: '100%',
                  padding: '10px 16px',
                  background: 'transparent',
                  border: 'none',
                  textAlign: 'left',
                  cursor: 'pointer',
                  fontSize: '14px',
                  color: 'var(--ifm-color-danger)',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '8px',
                }}
                onMouseEnter={(e) => e.currentTarget.style.background = 'var(--ifm-color-emphasis-100)'}
                onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
              >
                🚪 Logout
              </button>
            </div>
          )}
        </div>
      ) : (
        <button
          onClick={() => setIsOpen(true)}
          style={{
            position: 'fixed',
            top: '70px',
            right: '20px',
            padding: '8px 16px',
            background: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
            zIndex: 999,
            fontSize: '14px',
          }}
        >
          🔐 Login
        </button>
      )}

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
