\documentclass{article}
\usepackage[utf8]{inputenc}
\usepackage{amsmath, amssymb}
\usepackage{graphicx}
\usepackage{hyperref}
\usepackage{geometry}
\geometry{margin=1in}

\title{Sanctuary Take-Home Assignment\\Kinematics and Inverse Kinematics of a 3-DOF Planar Manipulator}
\author{Mahdi Torabi}
\date{\today}

\begin{document}

\maketitle

\section*{Introduction}
This project presents the implementation of forward and inverse kinematics for a 3-degree-of-freedom (3-DOF) planar robotic manipulator as part of a take-home assignment. I, \textbf{Mahdi Torabi}, designed and implemented this system in C++ using object-oriented programming and a modular architecture with full unit test coverage.

The manipulator consists of three links with lengths \( L_1, L_2, L_3 \) and three revolute joints represented by joint angles \( \theta_1, \theta_2, \theta_3 \).

\section*{Task Overview}
\begin{itemize}
    \item \textbf{Task 1:} Create an object-oriented model of the robot to store link parameters and joint angles.
    \item \textbf{Task 2:} Implement forward kinematics (FK) to compute end-effector position and orientation.
    \item \textbf{Task 3:} Implement inverse kinematics (IK):
        \begin{itemize}
            \item Algebraic solution
            \item Numerical methods: Gradient Descent, Levenberg-Marquardt, and Quadratic Programming
        \end{itemize}
    \item \textbf{Bonus:} Null-space optimization for redundancy resolution.
\end{itemize}

\section*{Forward Kinematics (FK)}
The forward kinematics calculates the end-effector pose \((x, y, \phi)\) given the joint angles \( \theta_1, \theta_2, \theta_3 \).

\subsection*{Equations:}
\begin{align*}
    x &= L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3) \\
    y &= L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3) \\
    \phi &= \theta_1 + \theta_2 + \theta_3
\end{align*}

\section*{Inverse Kinematics (IK)}
\subsection*{Algebraic Solution:}
The algebraic method solves the nonlinear system by:
\begin{enumerate}
    \item Computing wrist center:
    \begin{equation*}
        x_w = x - L_3 \cos(\phi), \quad y_w = y - L_3 \sin(\phi)
    \end{equation*}
    \item Solving for \( \theta_2 \):
    \begin{equation*}
        \cos(\theta_2) = \frac{x_w^2 + y_w^2 - L_1^2 - L_2^2}{2L_1 L_2}
    \end{equation*}
    \item Solving for \( \theta_1 \):
    \begin{equation*}
        \theta_1 = \arctan2(y_w, x_w) - \arctan2(L_2 \sin(\theta_2), L_1 + L_2 \cos(\theta_2))
    \end{equation*}
    \item Solving for \( \theta_3 \):
    \begin{equation*}
        \theta_3 = \phi - (\theta_1 + \theta_2)
    \end{equation*}
\end{enumerate}

\subsection*{Numerical Methods}
\subsubsection*{1. Gradient Descent}
We iteratively update the joint angles using:
\begin{equation*}
    \Delta \theta = \alpha J^T (x_{target} - x_{current})
\end{equation*}

\subsubsection*{2. Levenberg-Marquardt (Damped Least Squares)}
Improves over simple gradient descent:
\begin{equation*}
    \Delta \theta = (J^T J + \lambda I)^{-1} J^T (x_{target} - x_{current})
\end{equation*}

\subsubsection*{3. Quadratic Programming (QP)}
Formulated as an optimization problem:
\begin{align*}
    \min_{\Delta \theta} \quad & \frac{1}{2} \Delta \theta^T J^T J \Delta \theta + \Delta \theta^T J^T e \\
    \text{subject to:} \quad & \theta_{min} \leq \theta + \Delta \theta \leq \theta_{max}
\end{align*}
We use the OSQP solver to enforce joint limits and penalties on large steps.

\section*{Null-Space Optimization (Elbow-Up)}
In redundant manipulators, the null space of the Jacobian allows for secondary objectives. We used this idea to bias the solution towards an "elbow-up" configuration:
\begin{align*}
    \Delta \theta &= J^{\dagger} e + (I - J^{\dagger} J) \nabla h(\theta) \\
    h(\theta) &= \frac{1}{2} \|\theta - \theta_{rest}\|^2
\end{align*}
This was implemented using a virtual spring pulling towards a rest pose \( \theta_{rest} = [0, \pi/2, 0] \).

\section*{Project Structure}
\begin{itemize}
    \item \texttt{src/} - Implementation files
    \item \texttt{include/} - Headers
    \item \texttt{tests/} - Google Test unit tests
    \item \texttt{CMakeLists.txt} - Build configuration
    \item \texttt{osqp-cpp, osqp-eigen} - Libraries for QP solver
\end{itemize}

\section*{Running the Project}
\begin{enumerate}
    \item Clone the repo
    \item Install dependencies (Eigen, OSQP, GoogleTest)
    \item Run:
    \begin{verbatim}
    mkdir build && cd build
    cmake ..
    make
    ctest --output-on-failure
    \end{verbatim}
\end{enumerate}

\section*{Conclusion}
This project demonstrates a complete implementation of kinematics for a planar robotic arm using both analytical and numerical approaches. The evolution from algebraic to modern optimization-based methods (like QP with joint limits) shows a progressive improvement in capability, flexibility, and robustness.

\end{document}

