﻿using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace G__Interpreter
{
    /// <summary>
    /// Base class for all types of expressions.
    /// </summary>
    public abstract class Expression
    {
        //public abstract DataType Type { get; }
    }
    /// <summary>
    /// Represents a binary expression composed of a left expression, an operator token, and a right expression.
    /// </summary>
    public class BinaryExpression : Expression
    {
        public Expression Left { get; }
        public Token Operator { get; }
        public Expression Right { get; }

        public BinaryExpression(Expression left, Token op, Expression right)
        {
            Left = left;
            Operator = op;
            Right = right;
        }
    }

    /// <summary>
    /// Represents a unary expression composed of an operator token and a right expression.
    /// </summary>
    public class UnaryExpression : Expression
    {
        public Token Operator { get; }
        public Expression Right { get; }
        public UnaryExpression(Token op, Expression right)
        {
            Operator = op;
            Right = right;
        }
    }

    /// <summary>
    /// Represents a literal value expression, such as a number, boolean or string.
    /// </summary>
    public class LiteralExpression : Expression
    {
        public object Value { get; }

        public LiteralExpression(object value)
        {
            Value = value;
        }
    }

    /// <summary>
    /// Represents a grouping expression that wraps another expression.
    /// </summary>
    public class GroupingExpression : Expression
    {
        public Expression Expression { get; }

        public GroupingExpression(Expression expression)
        {
            Expression = expression;
        }
    }


    /// <summary>
    /// Represents a variable expression identified by a string.
    /// </summary>
    public class VariableExpression : Expression
    {
        public string ID { get; }

        public VariableExpression(string id)
        {
            ID = id;
        }
    }

    /// <summary>
    /// Represents an if-else statement with a condition expression, a then branch expression, and an optional else branch expression.
    /// </summary>
    public class Conditional : Expression
    {
        public Expression Condition { get; }
        public Expression ThenBranch { get; }
        public Expression ElseBranch { get; }

        public Conditional(Expression condition, Expression thenBranch, Expression elseBranch)
        {
            Condition = condition;
            ThenBranch = thenBranch;
            ElseBranch = elseBranch;
        }
    }

    /// <summary>
    /// Represents a let-in expression, where a set of instructions are executed before the body expression.
    /// </summary>
    public class LetExpression : Expression
    {
        public List<Expression> Instructions { get; }
        public Expression Body { get; }

        public LetExpression(List<Expression> instructions, Expression body)
        {
            Instructions = instructions;
            Body = body;
        }
    }
    /*public class LetInExpression : Expression
    {
        public List<AssignExpression> Assignments { get; }
        public Expression Body { get; }

        public LetInExpression(List<AssignExpression> assignments, Expression body)
        {
            Assignments = assignments;
            Body = body;
        }
    }*/

    /// <summary>
    /// Represents an assignment expression, where a value is assigned to a variable identified by a string.
    /// </summary>
    public class Assignment : Expression
    {
        public string ID { get; }
        public Expression Value { get; }

        public Assignment(string id, Expression value)
        {
            ID = id;
            Value = value;
        }
    }

    /// <summary>
    /// Represents a match-assignment expression, where elements of a sequence are assigned to a list of variables identified by a list of strings.
    /// </summary>
    public class MatchAssigment : Expression
    {
        public List<string> IDs { get; }
        public Sequence Sequence { get; }

        public MatchAssigment(List<string> ids, Sequence seq)
        {
            IDs = ids;
            Sequence = seq;
        }
    }

    /// <summary>
    /// Represents a function declaration with an identifier, a list of argument variable expressions, and a body expression.
    /// </summary>
    public class Function : Expression
    {
        public string Identifier { get; }
        public List<VariableExpression> Parameters { get; }
        public Expression Body { get; }

        public Function(string identifier, List<VariableExpression> parameters, Expression body)
        {
            Identifier = identifier;
            Parameters = parameters;
            Body = body;
        }
    }

    /// <summary>
    /// Represents a call to a function with an identifier and a list of argument expressions.
    /// </summary>
    public class Call : Expression
    {
        public string Identifier { get; }
        public List<Expression> Arguments { get; }

        public Call(string identifier, List<Expression> arguments)
        {
            Identifier = identifier;
            Arguments = arguments;
        }
    }
    /// <summary>
    /// Class that representes undefined or null expressions.
    /// </summary>
    public class Undefined : Expression
    {
        public Undefined() { }
    }

}
