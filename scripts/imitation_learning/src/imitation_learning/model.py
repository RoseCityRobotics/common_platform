import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models
from typing import Optional, Dict, Any
import math


class PositionalEncoding(nn.Module):
  """Sinusoidal positional encoding for transformer."""
  
  def __init__(self, d_model: int, max_len: int = 5000):
    super().__init__()
    
    pe = torch.zeros(max_len, d_model)
    position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
    div_term = torch.exp(torch.arange(0, d_model, 2).float() * 
                        (-math.log(10000.0) / d_model))
    
    pe[:, 0::2] = torch.sin(position * div_term)
    pe[:, 1::2] = torch.cos(position * div_term)
    pe = pe.unsqueeze(0).transpose(0, 1)
    
    self.register_buffer('pe', pe)
  
  def forward(self, x: torch.Tensor) -> torch.Tensor:
    """
    Args:
      x: (seq_len, batch, d_model)
    """
    return x + self.pe[:x.size(0), :]


class ActionEncoder(nn.Module):
  """
  Action encoder that encodes action chunks into latent Z variable (CVAE).
  
  Following ALOHA paper: q_φ(z|a_{t:t+k}, ō_t)
  Encodes action chunks conditioned on observations into a latent distribution.
  
  Note: In ALOHA, ō_t represents non-image observations. For our case,
  we condition on image features (temporal features from the vision encoder).
  """
  
  def __init__(
    self,
    action_dim: int = 2,
    chunk_size: int = 15,
    z_dim: int = 32,
    obs_dim: int = 512,  # Dimension of observation features (d_model)
    hidden_dim: int = 256
  ):
    super().__init__()
    self.action_dim = action_dim
    self.chunk_size = chunk_size
    self.z_dim = z_dim
    self.obs_dim = obs_dim
    
    # Flatten action chunk: (batch, chunk_size, action_dim) -> (batch, chunk_size * action_dim)
    action_input_dim = chunk_size * action_dim
    
    # Combine action and observation features
    # Use first timestep's observation features (following ALOHA's ō_t notation)
    combined_input_dim = action_input_dim + obs_dim
    
    # Encoder network (conditioned on both actions and observations)
    self.encoder = nn.Sequential(
      nn.Linear(combined_input_dim, hidden_dim),
      nn.ReLU(),
      nn.Linear(hidden_dim, hidden_dim),
      nn.ReLU()
    )
    
    # Output log-variance for latent Z (mean is fixed to 0, following ALOHA)
    # This reduces parameters and aligns with inference using z=0
    self.fc_logvar = nn.Linear(hidden_dim, z_dim)
  
  def forward(self, action_chunks: torch.Tensor, obs_features: torch.Tensor) -> tuple:
    """
    Encode action chunks conditioned on observations into latent Z distribution.
    
    Following ALOHA: z_mean is fixed to 0, only variance is learned.
    This reduces parameters and aligns with using z=0 during inference.
    
    Args:
      action_chunks: (batch, chunk_size, action_dim) - action chunks to encode
      obs_features: (batch, obs_dim) - observation features to condition on
                    Typically the temporal features from the first timestep
    
    Returns:
      z_mean: (batch, z_dim) - always zeros (fixed)
      z_logvar: (batch, z_dim) - log variance of latent distribution (learned)
      z: (batch, z_dim) - sampled latent variable (for training)
    """
    batch_size = action_chunks.shape[0]
    
    # Flatten action chunks
    action_flat = action_chunks.view(batch_size, -1)  # (batch, chunk_size * action_dim)
    
    # Concatenate actions and observations (CVAE conditioning)
    combined = torch.cat([action_flat, obs_features], dim=1)  # (batch, action_input_dim + obs_dim)
    
    # Encode
    hidden = self.encoder(combined)  # (batch, hidden_dim)
    
    # Get distribution parameters
    # z_mean is fixed to 0 (reduces parameters, aligns with inference)
    z_mean = torch.zeros(batch_size, self.z_dim, device=action_chunks.device)
    z_logvar = self.fc_logvar(hidden)  # (batch, z_dim)
    
    # Sample Z using reparameterization trick: z ~ N(0, exp(logvar))
    if self.training:
      std = torch.exp(0.5 * z_logvar)
      eps = torch.randn_like(std)
      z = z_mean + eps * std  # = eps * std (since z_mean = 0)
    else:
      # During inference, use z=0 (as per ALOHA Algorithm 2)
      z = z_mean  # = zeros
    
    return z_mean, z_logvar, z


class VisionEncoder(nn.Module):
  """Vision encoder using pretrained backbone."""
  
  def __init__(
    self,
    backbone: str = 'resnet18',
    pretrained: bool = True,
    feature_dim: int = 256,
    freeze_backbone: bool = False
  ):
    super().__init__()
    
    self.backbone_name = backbone
    self.feature_dim = feature_dim
    
    # Load pretrained backbone
    # Use 'weights' parameter instead of deprecated 'pretrained' (torchvision >= 0.13)
    # Fallback to 'pretrained' for older versions
    try:
      # Try new API (torchvision >= 0.13)
      if backbone == 'resnet18':
        weights = models.ResNet18_Weights.DEFAULT if pretrained else None
        self.backbone = models.resnet18(weights=weights)
        self.backbone.fc = nn.Identity()  # Remove final classification layer
        backbone_dim = 512
      elif backbone == 'resnet34':
        weights = models.ResNet34_Weights.DEFAULT if pretrained else None
        self.backbone = models.resnet34(weights=weights)
        self.backbone.fc = nn.Identity()
        backbone_dim = 512
      elif backbone == 'efficientnet_b0':
        weights = models.EfficientNet_B0_Weights.DEFAULT if pretrained else None
        self.backbone = models.efficientnet_b0(weights=weights)
        self.backbone.classifier = nn.Identity()
        backbone_dim = 1280
      else:
        raise ValueError(f"Unsupported backbone: {backbone}")
    except (AttributeError, TypeError):
      # Fallback to old API (torchvision < 0.13)
      if backbone == 'resnet18':
        self.backbone = models.resnet18(pretrained=pretrained)
        self.backbone.fc = nn.Identity()
        backbone_dim = 512
      elif backbone == 'resnet34':
        self.backbone = models.resnet34(pretrained=pretrained)
        self.backbone.fc = nn.Identity()
        backbone_dim = 512
      elif backbone == 'efficientnet_b0':
        self.backbone = models.efficientnet_b0(pretrained=pretrained)
        self.backbone.classifier = nn.Identity()
        backbone_dim = 1280
      else:
        raise ValueError(f"Unsupported backbone: {backbone}")
    
    # Projection layer to desired feature dimension
    self.projection = nn.Linear(backbone_dim, feature_dim)
    
    # Freeze backbone if specified
    if freeze_backbone:
      for param in self.backbone.parameters():
        param.requires_grad = False
  
  def forward(self, images: torch.Tensor) -> torch.Tensor:
    """
    Args:
      images: (batch, seq_len, 3, height, width)
    Returns:
      features: (seq_len, batch, feature_dim)
    """
    batch_size, seq_len = images.shape[:2]
    
    # Reshape to process all frames at once
    images_flat = images.view(batch_size * seq_len, *images.shape[2:])
    
    # Extract features
    with torch.no_grad() if not self.training else torch.enable_grad():
      features = self.backbone(images_flat)  # (batch * seq_len, backbone_dim)
    
    # Project to desired dimension
    features = self.projection(features)  # (batch * seq_len, feature_dim)
    
    # Reshape back to sequence format
    features = features.view(batch_size, seq_len, self.feature_dim)
    
    # Transpose for transformer: (seq_len, batch, feature_dim)
    features = features.transpose(0, 1)
    
    return features


class ActionChunkingTransformer(nn.Module):
  """
  Action Chunking Transformer for imitation learning with CVAE-style latent Z.
  
  Architecture (following ALOHA paper):
  1. Vision encoder: Extract features from image sequences
  2. Temporal encoder: Process temporal dependencies with transformer
  3. Action encoder: Encode action chunks conditioned on observations into latent Z 
                     (q_φ(z|a_{t:t+k}, ō_t)) - CVAE
  4. Action decoder: Predict action chunks using observations and Z (π_θ(â_{t:t+k}|o_t, z))
  """
  
  def __init__(
    self,
    # Vision encoder parameters
    backbone: str = 'resnet18',
    pretrained: bool = True,
    freeze_backbone: bool = False,
    
    # Transformer parameters
    d_model: int = 512,
    nhead: int = 4,
    num_encoder_layers: int = 6,
    num_decoder_layers: int = 6,
    dim_feedforward: int = 2048,
    dropout: float = 0.1,
    
    # Action prediction parameters
    chunk_size: int = 15,
    action_dim: int = 2,  # linear_x, angular_z
    
    # VAE parameters
    z_dim: int = 32,  # Dimension of latent Z variable
    
    # Other parameters
    max_seq_len: int = 100
  ):
    super().__init__()
    
    self.d_model = d_model
    self.chunk_size = chunk_size
    self.action_dim = action_dim
    self.z_dim = z_dim
    
    # Vision encoder
    self.vision_encoder = VisionEncoder(
      backbone=backbone,
      pretrained=pretrained,
      feature_dim=d_model,
      freeze_backbone=freeze_backbone
    )
    
    # Positional encoding
    self.pos_encoding = PositionalEncoding(d_model, max_seq_len)
    
    # Temporal transformer encoder
    encoder_layer = nn.TransformerEncoderLayer(
      d_model=d_model,
      nhead=nhead,
      dim_feedforward=dim_feedforward,
      dropout=dropout,
      batch_first=False
    )
    self.temporal_encoder = nn.TransformerEncoder(
      encoder_layer, 
      num_layers=num_encoder_layers,
      enable_nested_tensor=False  # Disable nested tensor to avoid warning
    )
    
    # Action encoder (for CVAE-style latent Z, conditioned on observations)
    self.action_encoder = ActionEncoder(
      action_dim=action_dim,
      chunk_size=chunk_size,
      z_dim=z_dim,
      obs_dim=d_model,  # Condition on temporal features (d_model dimension)
      hidden_dim=d_model
    )
    
    # Projection to incorporate Z into decoder queries
    # Z will be added to action queries
    self.z_projection = nn.Linear(z_dim, d_model)
    
    # Action decoder
    decoder_layer = nn.TransformerDecoderLayer(
      d_model=d_model,
      nhead=nhead,
      dim_feedforward=dim_feedforward,
      dropout=dropout,
      batch_first=False
    )
    self.action_decoder = nn.TransformerDecoder(
      decoder_layer,
      num_layers=num_decoder_layers
    )
    
    # Action queries (learnable embeddings for each future timestep)
    self.action_queries = nn.Parameter(
      torch.randn(chunk_size, d_model)
    )
    
    # Action prediction head
    self.action_head = nn.Sequential(
      nn.Linear(d_model, d_model // 2),
      nn.ReLU(),
      nn.Dropout(dropout),
      nn.Linear(d_model // 2, action_dim)
    )
    
    # Initialize weights
    self._init_weights()
  
  def _init_weights(self):
    """Initialize model weights."""
    for module in self.modules():
      if isinstance(module, nn.Linear):
        nn.init.xavier_uniform_(module.weight)
        if module.bias is not None:
          nn.init.constant_(module.bias, 0)
      elif isinstance(module, nn.LayerNorm):
        nn.init.constant_(module.bias, 0)
        nn.init.constant_(module.weight, 1.0)
  
  def forward(
    self, 
    images: torch.Tensor,
    actions: Optional[torch.Tensor] = None,
    z: Optional[torch.Tensor] = None,
    return_z_stats: bool = False,
    return_attention: bool = False
  ):
    """
    Forward pass with CVAE-style latent Z (conditioned on observations).
    
    Args:
      images: (batch, seq_len, 3, height, width) - input image sequence
      actions: (batch, seq_len, chunk_size, action_dim) - target actions (for training)
                If provided during training, Z will be encoded from actions conditioned on observations.
                If None, Z will be set to 0 (for inference).
      z: (batch, z_dim) - optional pre-computed latent Z variable
          If None and actions provided, Z will be encoded from actions and observations (CVAE).
          If None and actions not provided, Z=0 (inference mode).
      return_z_stats: Whether to return Z statistics (mean, logvar) for KL loss
      return_attention: Whether to return attention weights
    
    Returns:
      actions: (batch, seq_len, chunk_size, action_dim) - predicted action chunks
      z_stats: (z_mean, z_logvar) if return_z_stats=True, else None
    """
    batch_size, seq_len = images.shape[:2]
    
    # Vision encoding
    # (seq_len, batch, d_model)
    vision_features = self.vision_encoder(images)
    
    # Add positional encoding
    vision_features = self.pos_encoding(vision_features)
    
    # Temporal encoding
    # (seq_len, batch, d_model)
    temporal_features = self.temporal_encoder(vision_features)
    
    # Encode Z from actions conditioned on observations (CVAE, training mode)
    z_mean = None
    z_logvar = None
    if actions is not None and self.training:
      # Use first timestep's action chunk and observation features to encode Z
      # Following ALOHA: q_φ(z|a_{t:t+k}, ō_t)
      # In ALOHA, ō_t is non-image observations. For us, we use temporal features.
      action_chunk = actions[:, 0, :, :]  # (batch, chunk_size, action_dim)
      obs_features = temporal_features[0].transpose(0, 1)  # (batch, d_model) - first timestep
      z_mean, z_logvar, z = self.action_encoder(action_chunk, obs_features)
    elif z is not None:
      # Use provided Z
      pass
    else:
      # Inference mode: use Z=0 (as per ALOHA Algorithm 2)
      z = torch.zeros(batch_size, self.z_dim, device=images.device)
    
    # Project Z to match d_model dimension
    z_proj = self.z_projection(z)  # (batch, d_model)
    
    # Prepare action queries and add Z
    # (chunk_size, batch, d_model)
    action_queries = self.action_queries.unsqueeze(1).expand(-1, batch_size, -1)
    # Add Z to queries: broadcast (batch, d_model) -> (chunk_size, batch, d_model)
    action_queries = action_queries + z_proj.unsqueeze(0)
    
    # Action decoding
    # (chunk_size, batch, d_model)
    decoded_features = self.action_decoder(
      tgt=action_queries,
      memory=temporal_features
    )
    
    # Transpose for batch processing: (batch, chunk_size, d_model)
    decoded_features = decoded_features.transpose(0, 1)
    
    # Predict actions for each timestep
    # (batch, chunk_size, action_dim)
    actions_pred = self.action_head(decoded_features)
    
    # Expand to all timesteps: (batch, seq_len, chunk_size, action_dim)
    actions_pred = actions_pred.unsqueeze(1).expand(-1, seq_len, -1, -1)
    
    if return_z_stats:
      return actions_pred, (z_mean, z_logvar)
    elif return_attention:
      return actions_pred, None
    else:
      return actions_pred
  
  def predict_actions(
    self,
    images: torch.Tensor,
    use_first_action_only: bool = True,
    z: Optional[torch.Tensor] = None
  ) -> torch.Tensor:
    """
    Predict actions for inference.
    
    Args:
      images: (batch, seq_len, 3, height, width) - input image sequence
      use_first_action_only: If True, return only the first action of each chunk
      z: Optional latent Z variable. If None, uses Z=0 (as per ALOHA Algorithm 2)
    
    Returns:
      actions: (batch, seq_len, action_dim) or (batch, seq_len, chunk_size, action_dim)
    """
    self.eval()
    with torch.no_grad():
      # Get full action chunks (Z=0 for inference)
      action_chunks = self.forward(images, actions=None, z=z)  # (batch, seq_len, chunk_size, action_dim)
      
      if use_first_action_only:
        # Return only the first action of each chunk
        return action_chunks[:, :, 0, :]  # (batch, seq_len, action_dim)
      else:
        return action_chunks
  
  def get_model_info(self) -> Dict[str, Any]:
    """Get model information."""
    total_params = sum(p.numel() for p in self.parameters())
    trainable_params = sum(p.numel() for p in self.parameters() if p.requires_grad)
    
    return {
      'total_parameters': total_params,
      'trainable_parameters': trainable_params,
      'd_model': self.d_model,
      'chunk_size': self.chunk_size,
      'action_dim': self.action_dim,
      'backbone': self.vision_encoder.backbone_name,
      'num_encoder_layers': len(self.temporal_encoder.layers),
      'num_decoder_layers': len(self.action_decoder.layers)
    }


def create_model(config: Dict[str, Any]) -> ActionChunkingTransformer:
  """Create model from configuration."""
  model_config = config.get('model', {})
  
  model = ActionChunkingTransformer(
    backbone=model_config.get('backbone', 'resnet18'),
    pretrained=model_config.get('pretrained', True),
    freeze_backbone=model_config.get('freeze_backbone', False),
    d_model=model_config.get('d_model', 512),
    nhead=model_config.get('nhead', 4),
    num_encoder_layers=model_config.get('num_encoder_layers', 6),
    num_decoder_layers=model_config.get('num_decoder_layers', 6),
    dim_feedforward=model_config.get('dim_feedforward', 2048),
    dropout=model_config.get('dropout', 0.1),
    chunk_size=model_config.get('chunk_size', 15),
    action_dim=model_config.get('action_dim', 2),
    z_dim=model_config.get('z_dim', 32),  # Latent Z dimension
    max_seq_len=model_config.get('max_seq_len', 100)
  )
  
  return model


def load_pretrained_weights(
  model: ActionChunkingTransformer,
  checkpoint_path: str,
  device: str = 'cpu',
  strict: bool = True
) -> ActionChunkingTransformer:
  """
  Load pretrained weights from checkpoint.
  
  Args:
    model: Model to load weights into
    checkpoint_path: Path to checkpoint file
    device: Device to load on
    strict: Whether to strictly enforce key matching
  
  Returns:
    Model with loaded weights
  """
  checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
  
  if 'model_state_dict' in checkpoint:
    state_dict = checkpoint['model_state_dict']
  else:
    state_dict = checkpoint
  
  # Load state dict
  model.load_state_dict(state_dict, strict=strict)
  
  return model


def freeze_layers(model: ActionChunkingTransformer, layer_names: list):
  """
  Freeze specific layers by name.
  
  Args:
    model: Model to freeze layers in
    layer_names: List of layer names to freeze (e.g., ['vision_encoder', 'temporal_encoder'])
  """
  for name, param in model.named_parameters():
    for layer_name in layer_names:
      if name.startswith(layer_name):
        param.requires_grad = False
        break


def unfreeze_layers(model: ActionChunkingTransformer, layer_names: list):
  """
  Unfreeze specific layers by name.
  
  Args:
    model: Model to unfreeze layers in
    layer_names: List of layer names to unfreeze
  """
  for name, param in model.named_parameters():
    for layer_name in layer_names:
      if name.startswith(layer_name):
        param.requires_grad = True
        break


def load_aloha_weights(
  model: ActionChunkingTransformer,
  aloha_checkpoint_path: str,
  device: str = 'cpu',
  load_vision: bool = True,
  load_transformer: bool = True,
  strict: bool = False
) -> ActionChunkingTransformer:
  """
  Load ALOHA pretrained weights into our model.
  
  This is a simplified version. For full ALOHA weight loading,
  use the transfer_learning.py module.
  
  Args:
    model: Our model to load weights into
    aloha_checkpoint_path: Path to ALOHA checkpoint
    device: Device to load on
    load_vision: Whether to load vision encoder weights
    load_transformer: Whether to load transformer weights
    strict: Whether to strictly enforce key matching
  
  Returns:
    Model with loaded weights
  """
  try:
    from transfer_learning import load_aloha_weights as _load_aloha_weights
    return _load_aloha_weights(
      model, aloha_checkpoint_path, device, load_vision, load_transformer, strict
    )
  except ImportError:
    print("Warning: transfer_learning module not available. Loading checkpoint directly...")
    return load_pretrained_weights(model, aloha_checkpoint_path, device, strict)


def transfer_aloha_weights_practical(
  model: ActionChunkingTransformer,
  aloha_checkpoint_path: str,
  device: str = 'cpu'
) -> ActionChunkingTransformer:
  """
  Practical ALOHA weight transfer for differential drive robot.
  
  Transfers:
  - Vision encoder: Use first camera stream from ALOHA
  - Temporal encoder: Direct transfer (same architecture)
  - Action decoder: Keep random (different output space)
  
  Args:
    model: Our navigation model
    aloha_checkpoint_path: Path to ALOHA checkpoint
    device: Device to load on
  
  Returns:
    Model with transferred weights
  """
  print("Loading ALOHA checkpoint...")
  checkpoint = torch.load(aloha_checkpoint_path, map_location=device, weights_only=False)
  
  if 'model_state_dict' in checkpoint:
    aloha_state = checkpoint['model_state_dict']
  else:
    aloha_state = checkpoint
  
  our_state = model.state_dict()
  transferred_weights = {}
  
  # 1. Transfer vision encoder (use first camera stream)
  print("Transferring vision encoder from first ALOHA camera...")
  vision_transferred = 0
  for key, value in aloha_state.items():
    if key.startswith('vision_encoder.0.'):  # First camera stream
      new_key = key.replace('vision_encoder.0.', 'vision_encoder.')
      if new_key in our_state and value.shape == our_state[new_key].shape:
        transferred_weights[new_key] = value
        vision_transferred += 1
      else:
        print(f"  Skipping {key} -> {new_key} (shape mismatch)")
  
  # 2. Transfer temporal encoder (direct match)
  print("Transferring temporal encoder...")
  encoder_transferred = 0
  for key, value in aloha_state.items():
    if key.startswith('temporal_encoder.'):
      if key in our_state and value.shape == our_state[key].shape:
        transferred_weights[key] = value
        encoder_transferred += 1
      else:
        print(f"  Skipping {key} (shape mismatch)")
  
  # 3. Load transferred weights
  model.load_state_dict(transferred_weights, strict=False)
  
  print(f"Transfer complete:")
  print(f"  Vision encoder: {vision_transferred} weights transferred")
  print(f"  Temporal encoder: {encoder_transferred} weights transferred")
  print(f"  Action decoder: Random initialization (different output space)")
  
  return model