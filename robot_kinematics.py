import numpy as np
import matplotlib.pyplot as plt


# ==========================================
# 1. Custom Συνάρτηση Πολλαπλασιασμού Quaternions
# ==========================================
def quat_mult(a, b):
    """
    Πολλαπλασιασμός δύο quaternions a και b.
    Η μορφή τους είναι [w, x, y, z] (όπου w το πραγματικό μέρος).
    """
    w1, v1 = a[0], a[1:4]
    w2, v2 = b[0], b[1:4]

    # w = w1*w2 - dot(v1, v2)
    w = w1 * w2 - np.dot(v1, v2)
    # v = w1*v2 + w2*v1 + cross(v1, v2)
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)

    return np.array([w, v[0], v[1], v[2]])


# ==========================================
# 2. Δεδομένα Συστήματος (Γωνία & Μετατόπιση)
# ==========================================
theta = 2 * np.pi / 3  # Περιστροφή 120 μοιρών (2π/3)
t = np.array([0, 0, 1])  # Διάνυσμα μεταφοράς κάμερας

# Σημεία στο σύστημα της κάμερας (Pc)
Pc = np.array([
    [0.366, 1, 1.36],
    [-0.134, 1, 2.2321],
    [0.366, -1, 1.366]
])

# ==========================================
# 3. Δημιουργία Quaternions Περιστροφής
# ==========================================
q0 = np.cos(theta / 2)
qv = np.array([0, np.sin(theta / 2), 0])  # Περιστροφή γύρω από τον άξονα Y

q = np.concatenate(([q0], qv))  # Quaternion q
qc = np.concatenate(([q0], -qv))  # Συζυγές (Conjugate) qc = q*

# ==========================================
# 4. Μετασχηματισμός Σημείων στο Σύστημα του Ρομπότ
# ==========================================
Pw = np.zeros((3, 3))

print("--- Υπολογισμός Συντεταγμένων στο Σύστημα του Ρομπότ ---")
for i in range(3):
    # Μετατροπή του 3D σημείου σε καθαρό quaternion [0, x, y, z]
    p = np.concatenate(([0], Pc[i, :]))

    # Εφαρμογή περιστροφής: p_rot = q * p * q*
    p_rot = quat_mult(quat_mult(q, p), qc)

    # Προσθήκη μεταφοράς (translation)
    Pw[:, i] = p_rot[1:4] + t
    print(f"Σημείο P{i + 1}: {np.round(Pw[:, i], 3)}")

# ==========================================
# 5. 3D Γραφική Παράσταση
# ==========================================
fig = plt.figure('3D Robotic Kinematics', figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Εξαγωγή σημείων για εύκολη σχεδίαση
X, Y, Z = Pw[0, :], Pw[1, :], Pw[2, :]

# Σχεδίαση των σημείων (κόκκινοι κύκλοι)
ax.plot(X, Y, Z, 'ro', markersize=8, label='Waypoints (P1, P2, P3)')

# Σχεδίαση της τροχιάς / ευθειών
ax.plot([X[0], X[1]], [Y[0], Y[1]], [Z[0], Z[1]], 'b-', linewidth=2, label='Path 1')
ax.plot([X[1], X[2]], [Y[1], Y[2]], [Z[1], Z[2]], 'g-', linewidth=2, label='Path 2')

# Ρυθμίσεις γραφήματος
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('Robot 3D Path Generation using Quaternions')
ax.legend()
ax.grid(True)

# Κάνουμε τους άξονες ίσους (equal aspect ratio) για σωστή απεικόνιση 3D
max_range = np.array([X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]).max() / 2.0
mid_x = (X.max() + X.min()) * 0.5
mid_y = (Y.max() + Y.min()) * 0.5
mid_z = (Z.max() + Z.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()