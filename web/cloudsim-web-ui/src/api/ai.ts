import { postJson } from "./client";

export type AiDomainId = "scene.ops" | "robot.command" | "process.flow";

export type AiChatResult = {
  ok: boolean;
  assistantText?: string;
  reply?: string;
  error?: string;
};

export const aiChat = (message: string, domain?: AiDomainId) =>
  postJson<AiChatResult>("/api/ai/chat", { message, domain });
