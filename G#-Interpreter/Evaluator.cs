﻿using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Markup;

namespace G__Interpreter
{
    /// <summary>
    /// Represents an evaluator that evaluates the syntax tree generated by the parser and produces the corresponding runtime values.
    /// </summary>
    public class Evaluator
    {
        private Stack<Dictionary<string, object>> Scopes;   // The stack of scopes
        private readonly int CallLimit = 1000;              // The maximum amount of calls allowed
        private int Calls;                                  // The amount of calls made

        public Evaluator()
        {
            Scopes = new Stack<Dictionary<string, object>>();
            Scopes.Push(new Dictionary<string, object>());
        }
        /// <summary>
        /// Creates a new scope with the values of the current scope and pushes it to the stack of scopes.
        /// </summary>
        private void PushScope()
        {
            Dictionary<string, object> newScope = new Dictionary<string, object>();
            foreach (var keyvaluepair in CurrentScope())
                newScope[keyvaluepair.Key] = keyvaluepair.Value;
            Scopes.Push(newScope);
        }
        /// <summary>
        /// Removes the topmost scope from the stack of scopes.
        /// </summary>
        private void PopScope()
        {
            Scopes.Pop();
        }
        /// <summary>
        /// Gets the current scope from the stack of scopes.
        /// </summary>
        private Dictionary<string, object> CurrentScope()
        {
            return this.Scopes.Peek();
        }
        public List<object> Evaluate(List<Expression> AST)
        {
            List<object> Results = new List<object>();
            // Evaluate the expressions
            foreach (Expression expression in AST)
            {
                object result = Evaluate(expression);
                if (result is not null)
                    Results.Add(result);
            }
            return Results;
        }
        /// <summary>
        /// Evaluates the given expression and returns the result.
        /// </summary>
        /// <param name="expression">The expression to evaluate.</param>
        /// <returns>The result of the evaluation.</returns>
        public object Evaluate(Expression expression)
        {
            // Check if the amount of calls exceeds the call limit
            if (Calls > CallLimit)
                throw new Error(ErrorType.SEMANTIC, "Stack Overflow.");
            Calls++;

            // Evaluate the expression
            switch (expression)
            {
                case null:
                    return null;
                case LiteralExpression literal:
                    return literal.Value;
                case UnaryExpression unary:
                    return EvaluateUnary(unary.Operator, Evaluate(unary.Right));
                case BinaryExpression binary:
                    return EvaluateBinary(Evaluate(binary.Left), binary.Operator, Evaluate(binary.Right));
                case GroupingExpression grouping:
                    return Evaluate(grouping.Expression);
                case VariableExpression variable:
                    return EvaluateVariable(variable.ID);
                case AssignExpression assign:
                    return CurrentScope()[assign.ID] = Evaluate(assign.Value);
                case IfElseStatement ifElse:
                    return EvaluateIfElse(ifElse);
                case LetInExpression letIn:
                    return EvaluateLetIn(letIn);
                case FunctionDeclaration function:
                    return $"Function '{function.Identifier}' was declared succesfully.";
                case FunctionCall call:
                    return EvaluateFunction(call);
                case GeometricExpression geometric:
                    return geometric;
                default:
                    return null;
            }   
        }
        /// <summary>
        /// Evaluates a binary expression by performing the corresponding operation on the left and right operands.
        /// </summary>
        /// <param name="left">The left operand.</param>
        /// <param name="Operator">The binary operator.</param>
        /// <param name="right">The right operand.</param>
        /// <returns>The result of the binary operation.</returns>
        public object EvaluateBinary(object left, Token Operator, object right)
        {
            switch (Operator.Type)
            {
                case TokenType.ADDITION:
                    try
                    {
                        CheckNumbers(Operator, left, right);
                    }
                    catch(Error)
                    {
                        if (left is string || right is string)
                            return left.ToString() + right.ToString();
                        throw new Error(ErrorType.SEMANTIC, $"Operands must be Numbers or Strings in '{Operator.Lexeme}' operation.");
                    }
                    return (double)left + (double)right;


                case TokenType.SUBSTRACTION:
                    CheckNumbers(Operator, left, right);
                    return (double)left - (double)right;
                case TokenType.MULTIPLICATION:
                    CheckNumbers(Operator, left, right);
                    return (double)left * (double)right;
                case TokenType.DIVISION:
                    CheckNumbers(Operator, left, right);
                    if ((double)right != 0)
                        return (double)left / (double)right;
                    throw new Error(ErrorType.SEMANTIC, "Division by zero is undefined.");
                case TokenType.MODULO:
                    CheckNumbers(Operator, left, right);
                    return (double)left % (double)right;
                case TokenType.POWER:
                    CheckNumbers(Operator, left, right);
                    return Math.Pow((double)left, (double)right);
                case TokenType.GREATER:
                    CheckNumbers(Operator, left, right);
                    return (double)left > (double)right;
                case TokenType.GREATER_EQUAL:
                    CheckNumbers(Operator, left, right);
                    return (double)left >= (double)right;
                case TokenType.LESS:
                    CheckNumbers(Operator, left, right);
                    return (double)left < (double)right;
                case TokenType.LESS_EQUAL:
                    CheckNumbers(Operator, left, right);
                    return (double)left <= (double)right;

                case TokenType.EQUAL:
                    return IsEqual(left, right);
                case TokenType.NOT_EQUAL:
                    return !IsEqual(left, right);

                case TokenType.AND:
                    CheckBooleans(Operator, left, right);
                    return (bool)left && (bool)right;
                case TokenType.OR:
                    CheckBooleans(Operator, left, right);
                    return (bool)left || (bool)right;

                case TokenType.CONCAT:
                    return left.ToString() + right.ToString();

                default:
                    return null;
            }
        }
        /// <summary>
        /// Evaluates a unary expression by performing the corresponding operation on the right operand.
        /// </summary>
        /// <param name="Operator">The unary operator.</param>
        /// <param name="right">The right operand.</param>
        /// <returns>The result of the unary operation.</returns>
        public object EvaluateUnary(Token Operator, object right)
        {
            switch (Operator.Type)
            {
                case TokenType.NOT:
                    CheckBoolean(Operator, right);
                    return !(bool)right;
                case TokenType.SUBSTRACTION:
                    CheckNumber(Operator, right);
                    return -(double)right;
                default:
                    return null;
            }
        }
        /// <summary>
        /// Evaluates a variable expression by retrieving its value from the current scope.
        /// </summary>
        /// <param name="name">The name of the variable.</param>
        /// <returns>The value of the variable.</returns>
        public object EvaluateVariable(string name)
        {
            return CurrentScope().ContainsKey(name)? CurrentScope()[name] : throw new Error(ErrorType.SEMANTIC, $"Value of {name} wasn't declared.");
        }
        /// <summary>
        /// Evaluates a let-in expression by creating a new scope, declaring the variables and evaluating the body expression.
        /// </summary>
        /// <param name="letIn">The let-in expression to evaluate.</param>
        /// <returns>The result of the body expression.</returns>
        public object EvaluateLetIn(LetInExpression letIn)
        {
            PushScope();
            foreach (AssignExpression assign in letIn.Assignments)
            {
                object value = Evaluate(assign.Value);
                CurrentScope()[assign.ID] = value;
            }
            object result = Evaluate(letIn.Body);
            PopScope();
            return result;
        }
        /// <summary>
        /// Evaluates an if-else statement by evaluating the condition and either the then branch or the else branch based on the result.
        /// </summary>
        /// <param name="ifElse">The if-else statement to be evaluated.</param>
        /// <returns>The evaluated value of the executed branch.</returns>
        public object EvaluateIfElse(IfElseStatement ifElse)
        {
            object condition = Evaluate(ifElse.Condition);
            if (!IsBoolean(condition))
                throw new Error(ErrorType.SEMANTIC, "Condition in 'If-Else' expression must be a boolean expression.");
            return (bool)condition ? Evaluate(ifElse.ThenBranch) : Evaluate(ifElse.ElseBranch);
        }
        /// <summary>
        /// Evaluates a function call expression by evaluating the arguments and executing the function.
        /// </summary>
        /// <param name="call">The function call to evaluate.</param>
        /// <returns>The result of the evaluated function.</returns>
        public object EvaluateFunction(FunctionCall call)
        {
            // Check if the function was declared
            if (!StandardLibrary.DeclaredFunctions.ContainsKey(call.Identifier))
                throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' wasn't declared.");

            // Evaluate the arguments
            List<object> args = new List<object>();
            foreach (Expression arg in call.Arguments)
            {
                args.Add(Evaluate(arg));
            }
            switch (call.Identifier)
            {
                case "print":
                    if (args.Count != 1)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' received '{args.Count}' argument(s) instead of the correct amount '1'");
                    return args[0];
                case "sin":
                    if (args.Count != 1)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' received '{args.Count}' argument(s) instead of the correct amount '1'");
                    if (!IsNumber(args[0]))
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' can only receives 'Number'.");
                    return Math.Sin((double)args[0]);
                case "cos":
                    if (args.Count != 1)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' received '{args.Count}' argument(s) instead of the correct amount '1'"); ;
                    if (!IsNumber(args[0]))
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' can only receives 'Number'.");
                    return Math.Cos((double)args[0]);
                case "sqrt":
                    if (args.Count != 1)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' received '{args.Count}' argument(s) instead of the correct amount '1'");
                    if (!IsNumber(args[0]))
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' can only receives 'Number'.");
                    return Math.Sqrt((double)args[0]);
                case "log":
                    if (args.Count != 2)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' received '{args.Count}' argument(s) instead of the correct amount '2'");
                    if (!IsNumber(args[0], args[1]))
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' can only receives 'Number'.");
                    return Math.Log((double)args[0], (double)args[1]);
                case "exp":
                    if (args.Count != 1)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' received '{args.Count}' argument(s) instead of the correct amount '1'");
                    if (!IsNumber(args[0]))
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' can only receives 'Number'.");
                    return Math.Exp((double)args[0]);
                default:
                    // Get the function declaration
                    FunctionDeclaration function = StandardLibrary.DeclaredFunctions[call.Identifier];
                    // Check the amount of arguments of the function vs the arguments passed
                    if (args.Count != function.Parameters.Count)
                        throw new Error(ErrorType.SEMANTIC, $"Function '{call.Identifier}' receives '{args.Count}' argument(s) instead of the correct amount '{function.Parameters.Count}'");
                    PushScope();
                    // Add the evaluated arguments to the scope
                    for (int i = 0; i < function.Parameters.Count; i++)
                    {
                        string parameterName = function.Parameters[i].ID;
                        object argumentValue = args[i];
                        CurrentScope()[parameterName] = argumentValue;
                    }
                    // Evaluate the body of the function
                    object result = Evaluate(function.Body);
                    PopScope();
                    return result;
            }
        }

        #region Helper Methods

        /// <summary>
        /// Checks if the operand is a boolean value. Throws a semantic error if it is not.
        /// </summary>
        /// <param name="Operator">The operator token.</param>
        /// <param name="right">The operand to check.</param>
        public void CheckBoolean(Token Operator, object right)
        {
            if (IsBoolean(right)) return;
            throw new Error(ErrorType.SEMANTIC, $"Operand must be Boolean in '{Operator.Lexeme}' operation.");
        }
        /// <summary>
        /// Checks if the operands are boolean values. Throws a semantic error if they are not.
        /// </summary>
        /// <param name="Operator">The operator token.</param>
        /// <param name="left">The left operand to check.</param>
        /// <param name="right">The right operand to check.</param>
        public void CheckBooleans(Token Operator, object left, object right)
        {
            if (IsBoolean(left, right)) return;
            throw new Error(ErrorType.SEMANTIC, $"Operands must be Boolean in '{Operator.Lexeme}' operation.");
        }
        /// <summary>
        /// Checks if the operand is a number. Throws a semantic error if it is not.
        /// </summary>
        /// <param name="Operator">The operator token.</param>
        /// <param name="right">The operand to check.</param>
        public void CheckNumber(Token Operator, object right)
        {
            if (IsNumber(right)) return;
            throw new Error(ErrorType.SEMANTIC, $"Operand must be Number in '{Operator.Lexeme}' operation.");
        }
        /// <summary>
        /// Checks if the operands are numbers. Throws a semantic error if they are not.
        /// </summary>
        /// <param name="Operator">The operator token.</param>
        /// <param name="left">The left operand to check.</param>
        /// <param name="right">The right operand to check.</param>
        public void CheckNumbers(Token Operator, object left, object right)
        {
            if (IsNumber(left, right)) return;
            throw new Error(ErrorType.SEMANTIC, $"Operands must be Numbers in '{Operator.Lexeme}' operation.");
        }
        /// <summary>
        /// Checks if the given operands are number values.
        /// </summary>
        /// <param name="operands">The operands to check.</param>
        /// <returns><c>true</c> if all operands are numbers; otherwise, <c>false</c>.</returns>
        public bool IsNumber(params object[] operands)
        {
            foreach (object operand in operands)
            {
                if (operand is not double)
                    return false;
            }
            return true;
        }
        /// <summary>
        /// Checks if the given operands are boolean values.
        /// </summary>
        /// <param name="operands">The operands to check.</param>
        /// <returns><c>true</c> if all operands are booleans; otherwise, <c>false</c>.</returns>
        public bool IsBoolean(params object[] operands)
        {
            foreach (object operand in operands)
            {
                if (operand is not bool)
                    return false;
            }
            return true;
        }
        /// <summary>
        /// Checks if the left and right operands are equal.
        /// </summary>
        /// <param name="left">The left operand.</param>
        /// <param name="right">The right operand.</param>
        /// <returns><c>true</c> if the operands are equal; otherwise, <c>false</c>.</returns>
        public bool IsEqual(object left, object right)
        {
            if (left == null && right == null)
                return true;
            return left == null? false : left.Equals(right);
        }
        #endregion
    }
}