import React from 'react';
import {
  SignIn,
  SignInButton,
  useSignIn,
  useClerk,
  useUser
} from '@clerk/clerk-react';
import { useNavigate } from 'react-router-dom';

interface SignInModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const SignInModal: React.FC<SignInModalProps> = ({ isOpen, onClose }) => {
  if (!isOpen) return null;

  return (
    <div className="modal-overlay" style={{
      position: 'fixed',
      top: 0,
      left: 0,
      right: 0,
      bottom: 0,
      backgroundColor: 'rgba(0, 0, 0, 0.5)',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 1000,
      padding: '20px'
    }} onClick={onClose}>
      <div className="modal-content" style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        maxWidth: '400px',
        width: '100%',
        boxShadow: '0 10px 25px rgba(0,0,0,0.1)',
        overflow: 'hidden'
      }} onClick={(e) => e.stopPropagation()}>
        <div className="modal-header" style={{
          padding: '20px',
          borderBottom: '1px solid #e5e7eb',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center'
        }}>
          <h2 style={{ margin: 0, fontSize: '1.25rem', fontWeight: '600' }}>Sign In</h2>
          <button
            onClick={onClose}
            style={{
              background: 'none',
              border: 'none',
              fontSize: '1.5rem',
              cursor: 'pointer',
              color: '#6b7280'
            }}
          >
            &times;
          </button>
        </div>
        <div className="modal-body" style={{
          padding: '20px'
        }}>
          <SignIn
            appearance={{
              elements: {
                footer: {
                  display: 'none'
                },
                card: {
                  boxShadow: 'none',
                  border: 'none',
                  padding: 0
                }
              }
            }}
          />
        </div>
      </div>
    </div>
  );
};

export default SignInModal;